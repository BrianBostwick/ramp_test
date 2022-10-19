//! Single particle in a cross beam optical dipole trap
extern crate atomecs as lib;
extern crate nalgebra;
use lib::atom::{Atom, Force, Mass, Position, Velocity};
use lib::dipole::{self, DipolePlugin};
use lib::integrator::Timestep;
use lib::laser::{self, LaserPlugin};
use lib::laser::gaussian::GaussianBeam;
use lib::laser::intensity::{LaserIntensitySamplers};
use lib::output::file::{FileOutputPlugin, Text, XYZ};
use lib::simulation::SimulationBuilder;
use nalgebra::Vector3;
use specs::prelude::*;
use std::time::Instant;
use rand_distr::{Distribution, Normal};
use lib::initiate::NewlyCreated;
use std::fs::File;
use std::io::{Error, Write};
use lib::collisions::{CollisionPlugin, ApplyCollisionsOption, CollisionParameters, CollisionsTracker};
use lib::sim_region::{ SimulationVolume, VolumeType};
use lib::shapes::Sphere;

use lib::ramp::{Lerp, Ramp, RampUpdateSystem};

use specs::{Component, HashMapStorage};

const BEAM_NUMBER: usize = 2;

fn main() {
    let now = Instant::now();

    // Create dipole laser.
    // let mut power = 7.0;
    let e_radius = 60.0e-6 / 2.0_f64.sqrt();
    let wavelength = 1064.0e-9;


    let gaussian_beam_one = GaussianBeam {
        intersection: Vector3::new(0.0, 0.0, 0.0),
        e_radius,
        power: 7.0,
        direction: Vector3::x(),
        rayleigh_range: crate::laser::gaussian::calculate_rayleigh_range(&wavelength, &e_radius),
        ellipticity: 0.0,
    };

    // Configure simulation output.
    let mut sim_builder = SimulationBuilder::default();

    sim_builder.world.register::<NewlyCreated>();
    // sim_builder.world.register::<RampUpdateSystem>();

    sim_builder.add_plugin(LaserPlugin::<{BEAM_NUMBER}>);
    sim_builder.add_plugin(DipolePlugin::<{BEAM_NUMBER}>);


    sim_builder.add_plugin(FileOutputPlugin::<Position, Text, Atom>::new("D:/data_1/pos_test.txt".to_string(), 100));
    sim_builder.add_plugin(FileOutputPlugin::<Velocity, Text, Atom>::new("D:/data_1/vel_test.txt".to_string(), 100));
    sim_builder.add_plugin(FileOutputPlugin::<LaserIntensitySamplers<{BEAM_NUMBER}>, Text, LaserIntensitySamplers<{BEAM_NUMBER}>>::new("D:/data_1/intensity_test.txt".to_string(), 100));

    sim_builder.dispatcher_builder.with(
        RampUpdateSystem::<GaussianBeam>::default(),
        "update_comp",
        &[],
    );
    
    let mut sim = sim_builder.build();

    let frames = vec![
        (0.0, GaussianBeam { intersection: Vector3::new(0.0, 0.0, 0.0),
        e_radius,
        power: 7.0,
        direction: Vector3::x(),
        rayleigh_range: crate::laser::gaussian::calculate_rayleigh_range(&wavelength, &e_radius),
        ellipticity: 0.0 }),
        (1.0, GaussianBeam { intersection: Vector3::new(0.0, 0.0, 0.0),
        e_radius,
        power: 0.0,
        direction: Vector3::x(),
        rayleigh_range: crate::laser::gaussian::calculate_rayleigh_range(&wavelength, &e_radius),
        ellipticity: 0.0 }
        )
        ];

    let ramp = Ramp {
        prev: 0,
        keyframes: frames,
    };

    sim.world
        .create_entity()
        .with(gaussian_beam)
        .with(dipole::DipoleLight { wavelength })
        .with(laser::frame::Frame {
            x_vector: Vector3::y(),
            y_vector: Vector3::z(),
        })
        .with(ramp)
        .build();


    // Create a single test atom
    let atom_number = 25;
    for _ in 0..atom_number {
        sim.world
            .create_entity()
            .with(Atom)
            .with(Mass { value: 87.0 })
            .with(Force::new())
            .with(Position {
                pos: Vector3::new(
                    50e-6),
                    50e-6),
                    50e-6),
                ),
            })
            .with(Velocity {
                vel: Vector3::new(
                    0.0,
                    0.0,
                    0.0,
                ),
            })
            .with(dipole::Polarizability::calculate_for(
                wavelength, 461e-9, 32.0e6,
            ))
            .with(lib::initiate::NewlyCreated)
            .build();
    }


    // Define timestep
    sim.world.insert(Timestep { delta: 1.0e-6 });

    // Run the simulation for a number of steps.
    for _i in 0..500 {
        sim.step();
    }
    println!("Simulation completed in {} ms.", now.elapsed().as_millis());
}
