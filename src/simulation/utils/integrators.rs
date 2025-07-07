// src/utils/integrators.rs

use nalgebra::DVector;

pub trait Integrator<T> {
    fn step(
        &self,
        func: &dyn Fn(&DVector<T>, T) -> DVector<T>,
        x0: &DVector<T>,
        t0: T,
        tf: T,
    ) -> DVector<T>;
}

// Runge-Kutta methods
#[derive(Default)]
pub struct RK1;

impl<T> Integrator<T> for RK1
where
    T: Copy
        + std::ops::Sub<Output = T>
        + std::ops::Mul<DVector<T>, Output = DVector<T>>
        + num_traits::Float,
    DVector<T>: std::ops::Add<Output = DVector<T>>,
{
    fn step(
        &self,
        func: &dyn Fn(&DVector<T>, T) -> DVector<T>,
        x0: &DVector<T>,
        t0: T,
        tf: T,
    ) -> DVector<T> {
        let dt: T = tf - t0; // Calculate the time step
        x0.clone() + dt * func(x0, t0) // Euler's method
    }
}

#[derive(Default)]
pub struct RK2;

impl<T> Integrator<T> for RK2
where
    T: Copy
        + std::ops::Sub<Output = T>
        + std::ops::Mul<DVector<T>, Output = DVector<T>>
        + num_traits::Float,
    DVector<T>: std::ops::Add<Output = DVector<T>>,
{
    fn step(
        &self,
        func: &dyn Fn(&DVector<T>, T) -> DVector<T>,
        x0: &DVector<T>,
        t0: T,
        tf: T,
    ) -> DVector<T> {
        let dt: T = tf - t0; // Calculate the time step
        let k1: DVector<T> = func(x0, t0);
        let k2: DVector<T> = func(&(x0.clone() + dt * k1.clone()), t0 + dt);

        // Weighted average of k1 and k2
        x0.clone() + T::from(0.5).unwrap() * dt * (k1 + k2)
    }
}
#[derive(Default)]
pub struct RK3;

impl<T> Integrator<T> for RK3
where
    T: Copy
        + std::ops::Sub<Output = T>
        + std::ops::Mul<DVector<T>, Output = DVector<T>>
        + num_traits::Float,
    DVector<T>: std::ops::Add<Output = DVector<T>>,
{
    fn step(
        &self,
        func: &dyn Fn(&DVector<T>, T) -> DVector<T>,
        x0: &DVector<T>,
        t0: T,
        tf: T,
    ) -> DVector<T> {
        let dt: T = tf - t0; // Calculate the time step
        let k1: DVector<T> = func(x0, t0);
        let k2: DVector<T> = func(&(x0.clone() + dt * k1.clone()), t0 + dt);
        let k3: DVector<T> = func(
            &(x0.clone() + (dt / T::from(4).unwrap()) * (k1.clone() + k2.clone())),
            t0 + dt / T::from(2).unwrap(),
        );

        // Weighted average of k1, k2, and k3
        x0.clone() + (dt / T::from(6).unwrap()) * (k1 + k2 + T::from(4).unwrap() * k3)
    }
}

#[derive(Default)]
pub struct RK4;

impl<T> Integrator<T> for RK4
where
    T: Copy + num_traits::Float + std::ops::Mul<DVector<T>, Output = DVector<T>>,
    DVector<T>: std::ops::Add<Output = DVector<T>>,
{
    fn step(
        &self,
        func: &dyn Fn(&DVector<T>, T) -> DVector<T>,
        x0: &DVector<T>,
        t0: T,
        tf: T,
    ) -> DVector<T> {
        let dt = tf - t0;
        let half = T::from(0.5).unwrap();
        let sixth = T::from(1.0 / 6.0).unwrap();
        let two = T::from(2.0).unwrap();

        let k1 = func(x0, t0);
        let k2 = func(&(x0.clone() + half * dt * k1.clone()), t0 + half * dt);
        let k3 = func(&(x0.clone() + half * dt * k2.clone()), t0 + half * dt);
        let k4 = func(&(x0.clone() + dt * k3.clone()), tf);

        x0.clone() + dt * sixth * (k1 + two * k2 + two * k3 + k4)
    }
}
