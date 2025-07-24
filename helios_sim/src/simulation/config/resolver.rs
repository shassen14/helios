// // helios_sim/src/simulation/config/resolver.rs

// use super::catalog::PrefabCatalog;
// use figment::value::{Dict, Tag, Value};

// // The main public entry point.
// pub fn resolve_agent_value(agent_value: &Value, catalog: &PrefabCatalog) -> Result<Value, String> {
//     resolve_value_recursively(agent_value, catalog)
// }

// fn resolve_value_recursively(value: &Value, catalog: &PrefabCatalog) -> Result<Value, String> {
//     // Is the current value a dictionary with a 'from' key? This is our primary composition pattern.
//     if let Some(dict) = value.as_dict() {
//         if let Some(from_key) = dict.get("from").and_then(|v| v.as_str()) {
//             // 1. Fetch and COMPLETELY resolve the base prefab first.
//             let base_prefab_data = catalog
//                 .0
//                 .get(from_key)
//                 .ok_or_else(|| format!("Prefab '{}' not found in catalog", from_key))?;
//             let mut modified_base = resolve_value_recursively(base_prefab_data, catalog)?;

//             // 2. Sequentially apply modifications from the current level.

//             // Handle Dictionary Overrides
//             if let Some(override_value) = dict.get("override") {
//                 if let Some(override_dict) = override_value.as_dict() {
//                     // Try to apply a dictionary override.
//                     if let Some(base_dict_ref) = modified_base.as_dict() {
//                         let mut new_dict = base_dict_ref.clone();
//                         for (key, val) in override_dict.iter() {
//                             let resolved_val = resolve_value_recursively(val, catalog)?;
//                             new_dict.insert(key.clone(), resolved_val);
//                         }
//                         modified_base = Value::Dict(Tag::Default, new_dict);
//                     }
//                     // Try to apply a named-item override on a list.
//                     else if let Some(base_list_ref) = modified_base.as_array() {
//                         let mut new_list = base_list_ref.to_vec();
//                         for item_value in new_list.iter_mut() {
//                             if let Some(item_dict) = item_value.as_dict() {
//                                 if let Some(item_name) =
//                                     item_dict.get("name").and_then(|v| v.as_str())
//                                 {
//                                     if let Some(specific_override) = override_dict.get(item_name) {
//                                         // To merge, we must treat the item as a `{from, override}` block.
//                                         let temp_comp = Value::Dict(
//                                             Tag::Default,
//                                             Dict::from([
//                                                 ("from".to_string(), item_value.clone()),
//                                                 ("override".to_string(), specific_override.clone()),
//                                             ]),
//                                         );
//                                         *item_value =
//                                             resolve_value_recursively(&temp_comp, catalog)?;
//                                     }
//                                 }
//                             }
//                         }
//                         modified_base = Value::Array(Tag::Default, new_list);
//                     }
//                 }
//             }

//             // Handle List Additions
//             if let Some(add_value) = dict.get("add") {
//                 if let Some(base_list_ref) = modified_base.as_array() {
//                     let mut new_list = base_list_ref.to_vec();
//                     if let Some(items_to_add) = add_value.as_array() {
//                         for item in items_to_add {
//                             new_list.push(resolve_value_recursively(item, catalog)?);
//                         }
//                     }
//                     modified_base = Value::Array(Tag::Default, new_list);
//                 } else {
//                     return Err(format!("Cannot use 'add' on prefab '{}' because its base does not resolve to a list.", from_key));
//                 }
//             }

//             return Ok(modified_base);
//         }
//     }

//     // Unwrapping logic for `prefab_type = "list"`
//     if let Some(dict) = value.as_dict() {
//         if dict.get("prefab_type").and_then(|v| v.as_str()) == Some("list") {
//             let items = dict
//                 .get("items")
//                 .cloned()
//                 .ok_or_else(|| "List prefab is missing 'items' key".to_string())?;
//             return resolve_value_recursively(&items, catalog);
//         }
//     }

//     // Default recursion for children if no composition patterns match.
//     match value {
//         Value::Dict(tag, dict) => {
//             let mut new_dict = Dict::new();
//             for (key, val) in dict.iter() {
//                 new_dict.insert(key.clone(), resolve_value_recursively(val, catalog)?);
//             }
//             Ok(Value::Dict(tag.clone(), new_dict))
//         }
//         Value::Array(tag, arr) => {
//             let mut resolved_arr = Vec::new();
//             for item in arr {
//                 resolved_arr.push(resolve_value_recursively(item, catalog)?);
//             }
//             Ok(Value::Array(tag.clone(), resolved_arr))
//         }
//         _ => Ok(value.clone()),
//     }
// }

// helios_sim/src/simulation/config/resolver.rs

// use super::catalog::PrefabCatalog;
// use figment::value::{Dict, Tag, Value};

// pub fn resolve_agent_value(agent_value: &Value, catalog: &PrefabCatalog) -> Result<Value, String> {
//     let bob = resolve_value_recursively(agent_value, catalog);
//     println! {"{:?}", bob};
//     return bob;
// }

// /// A correct, recursive deep merge helper.
// fn deep_merge(base: &mut Dict, override_dict: &Dict) {
//     for (key, override_val) in override_dict {
//         // We do not want to merge the 'from' key itself, it's metadata.
//         if key == "from" {
//             continue;
//         }

//         if let Some(base_val) = base.get_mut(key) {
//             // If the key exists in the base and both values are dictionaries, we must recurse.
//             if let (Some(base_sub_dict), Some(override_sub_dict)) =
//                 (base_val.as_dict(), override_val.as_dict())
//             {
//                 let mut new_sub_dict = base_sub_dict.clone();
//                 deep_merge(&mut new_sub_dict, override_sub_dict);
//                 *base_val = Value::Dict(Tag::Default, new_sub_dict);
//                 continue;
//             }
//         }
//         // Otherwise (key doesn't exist in base, or values are not both dicts),
//         // we simply insert/overwrite the value from the override.
//         base.insert(key.clone(), override_val.clone());
//     }
// }

// fn resolve_value_recursively(value: &Value, catalog: &PrefabCatalog) -> Result<Value, String> {
//     // --- STEP 1: Process the current node if it's a `{ from = ... }` reference. ---
//     // This is a "pre-order" traversal. We resolve the parent before its children.
//     let current_value = if let Some(dict) = value.as_dict() {
//         if let Some(from_key) = dict.get("from").and_then(|v| v.as_str()) {
//             let base_prefab_data = catalog
//                 .0
//                 .get(from_key)
//                 .ok_or_else(|| format!("Prefab '{}' not found in catalog", from_key))?;

//             // Recursively resolve the base prefab first to get its complete structure.
//             let resolved_base = resolve_value_recursively(base_prefab_data, catalog)?;

//             // The base of a composition must be a dictionary.
//             let mut final_dict = resolved_base.into_dict().ok_or_else(|| {
//                 format!(
//                     "Prefab '{}' must resolve to a dictionary to be merged.",
//                     from_key
//                 )
//             })?;

//             // Use our deep_merge helper to apply the current level's overrides.
//             // The `dict` here contains the `from` key and any sibling override keys.
//             deep_merge(&mut final_dict, dict);

//             Value::Dict(Tag::Default, final_dict)
//         } else {
//             // Not a 'from' reference, so just use the original value.
//             value.clone()
//         }
//     } else {
//         // Not a dictionary, so it can't be a 'from' reference.
//         value.clone()
//     };

//     // --- STEP 2: Now that the current node is resolved, resolve its children. ---
//     match current_value {
//         Value::Dict(tag, dict) => {
//             let mut new_dict = Dict::new();
//             for (key, val) in dict.iter() {
//                 // The 'from' key has been processed, so we strip it from the final output.
//                 if key == "from" {
//                     continue;
//                 }
//                 new_dict.insert(key.clone(), resolve_value_recursively(val, catalog)?);
//             }
//             Ok(Value::Dict(tag.clone(), new_dict))
//         }
//         Value::Array(tag, arr) => {
//             let mut resolved_arr = Vec::new();
//             for item in arr {
//                 resolved_arr.push(resolve_value_recursively(&item, catalog)?);
//             }
//             Ok(Value::Array(tag.clone(), resolved_arr))
//         }
//         // Leaf nodes (strings, numbers) have no children.
//         _ => Ok(current_value),
//     }
// }

// helios_sim/src/simulation/config/resolver.rs

use super::catalog::PrefabCatalog;
use figment::{
    providers::Serialized,
    value::{Dict, Tag, Value},
    Figment,
};

pub fn resolve_agent_value(agent_value: &Value, catalog: &PrefabCatalog) -> Result<Value, String> {
    let bob = resolve_value_recursively(agent_value, catalog);
    println!("{:?}", bob);
    bob
}

/// A correct, recursive deep merge helper.
/// It merges the contents of `override_dict` into `base`.
fn deep_merge(base: &mut Dict, override_dict: &Dict) {
    for (key, override_val) in override_dict {
        // --- THIS IS THE FIX ---
        // If the override value is a dictionary containing a 'from' key,
        // it signifies a complete replacement, not a merge.
        if let Some(d) = override_val.as_dict() {
            if d.contains_key("from") {
                base.insert(key.clone(), override_val.clone());
                continue; // Skip to the next key
            }
        }

        // The original deep merge logic for all other cases.
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
    // This function now follows a strict "pre-order" traversal.
    // 1. Resolve the current node.
    // 2. Then, resolve the children of the result.

    // --- STEP 1: Resolve the current node if it's a 'from' reference. ---
    let current_node = if let Some(dict) = value.as_dict() {
        if let Some(from_key) = dict.get("from").and_then(|v| v.as_str()) {
            let base_prefab_data = catalog
                .0
                .get(from_key)
                .ok_or_else(|| format!("Prefab '{}' not found in catalog", from_key))?;

            // Recursively resolve the base prefab first to get its complete structure.
            let resolved_base = resolve_value_recursively(base_prefab_data, catalog)?;

            let mut final_dict = resolved_base.into_dict().ok_or_else(|| {
                format!(
                    "Prefab '{}' must resolve to a dictionary to be merged.",
                    from_key
                )
            })?;

            // Use our deep_merge helper to apply the current level's overrides.
            // The `dict` here contains the `from` key and any sibling override keys.
            deep_merge(&mut final_dict, dict);

            Value::Dict(Tag::Default, final_dict)
        } else {
            // Not a 'from' reference, just use the original value.
            value.clone()
        }
    } else {
        // Not a dictionary, so it can't be a 'from' reference.
        value.clone()
    };

    // --- STEP 2: Now that the current node is resolved, resolve its children. ---
    match &current_node {
        Value::Dict(tag, dict) => {
            let mut new_dict = Dict::new();
            for (key, val) in dict.iter() {
                // The 'from' key has been processed, so we strip it from the final output.
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
