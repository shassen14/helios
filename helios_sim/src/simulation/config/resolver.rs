// helios_sim/src/simulation/config/resolver.rs

use super::catalog::PrefabCatalog;
use figment::value::{Dict, Tag, Value};

pub fn resolve_agent_value(agent_value: &Value, catalog: &PrefabCatalog) -> Result<Value, String> {
    resolve_value_recursively(agent_value, catalog)
}

/// Recursively merges `override_dict` into `base`.
/// If an override value contains a `from` key, it replaces rather than merges.
fn deep_merge(base: &mut Dict, override_dict: &Dict) {
    for (key, override_val) in override_dict {
        if let Some(d) = override_val.as_dict() {
            if d.contains_key("from") {
                base.insert(key.clone(), override_val.clone());
                continue;
            }
        }

        if let Some(base_val) = base.get_mut(key) {
            if let (Some(base_sub_dict), Some(override_sub_dict)) =
                (base_val.as_dict(), override_val.as_dict())
            {
                let mut new_sub_dict = base_sub_dict.clone();
                deep_merge(&mut new_sub_dict, override_sub_dict);
                *base_val = Value::Dict(Tag::Default, new_sub_dict);
                continue;
            }
        }
        base.insert(key.clone(), override_val.clone());
    }
}

fn resolve_value_recursively(value: &Value, catalog: &PrefabCatalog) -> Result<Value, String> {
    // Step 1: Resolve the current node if it's a 'from' reference.
    let current_node = if let Some(dict) = value.as_dict() {
        if let Some(from_key) = dict.get("from").and_then(|v| v.as_str()) {
            let base_prefab_data = catalog
                .0
                .get(from_key)
                .ok_or_else(|| format!("Prefab '{}' not found in catalog", from_key))?;

            let resolved_base = resolve_value_recursively(base_prefab_data, catalog)?;

            let mut final_dict = resolved_base.into_dict().ok_or_else(|| {
                format!(
                    "Prefab '{}' must resolve to a dictionary to be merged.",
                    from_key
                )
            })?;

            deep_merge(&mut final_dict, dict);
            Value::Dict(Tag::Default, final_dict)
        } else {
            value.clone()
        }
    } else {
        value.clone()
    };

    // Step 2: Resolve children, stripping the consumed 'from' key.
    match &current_node {
        Value::Dict(tag, dict) => {
            let mut new_dict = Dict::new();
            for (key, val) in dict.iter() {
                if key == "from" {
                    continue;
                }
                new_dict.insert(key.clone(), resolve_value_recursively(val, catalog)?);
            }
            Ok(Value::Dict(tag.clone(), new_dict))
        }
        Value::Array(tag, arr) => {
            let mut resolved_arr = Vec::new();
            for item in arr {
                resolved_arr.push(resolve_value_recursively(item, catalog)?);
            }
            Ok(Value::Array(tag.clone(), resolved_arr))
        }
        _ => Ok(current_node),
    }
}
