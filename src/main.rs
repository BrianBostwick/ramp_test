
extern crate atomecs as lib;

use lib::atom::{Atom, Force, Mass, Position, Velocity};

use lib::integrator::Timestep;
use lib::output::file::{FileOutputPlugin, Text};
use lib::simulation::{SimulationBuilder};
use nalgebra::Vector3;
use specs::prelude::*;
use std::time::Instant;
use lib::initiate::NewlyCreated;

use lib::laser::{self, LaserPlugin};
use lib::laser::gaussian::GaussianBeam;
use lib::dipole::{self, DipolePlugin};

use lib::laser::intensity::{LaserIntensitySamplers};

use lib::ramp::{Ramp, RampUpdateSystem};

const BEAM_NUMBER: usize = 1;

fn main() {

    let now = Instant::now();

    // Configure simulation output.
    let mut sim_builder = SimulationBuilder::default();

    sim_builder.world.register::<NewlyCreated>();

    sim_builder.add_plugin(LaserPlugin::<{BEAM_NUMBER}>);
    sim_builder.add_plugin(DipolePlugin::<{BEAM_NUMBER}>);

    sim_builder.add_plugin(FileOutputPlugin::<Position, Text, Atom>::new("D:/data_1/pos_test.txt".to_string(), 1));
    sim_builder.add_plugin(FileOutputPlugin::<Velocity, Text, Atom>::new("D:/data_1/vel_test.txt".to_string(), 1));
    sim_builder.add_plugin(FileOutputPlugin::<
                LaserIntensitySamplers<{BEAM_NUMBER}>,
                Text,
                LaserIntensitySamplers<{BEAM_NUMBER}>>::new("D:/data_1/intensity_test.txt".to_string(),
            1
        )
    );

    // Must use .add() to add the ramp update system to the dispatcher
    sim_builder.dispatcher_builder.add(
        RampUpdateSystem::<GaussianBeam>::default(),
        "update_comp",
        &[],
    );


    let mut sim = sim_builder.build();
                              //units
    let wavelength = 1064e-9; //m
    let e_radius   = 50.0e-6; //m
    let dt         = 1.0e-6;  //s
    let sim_length = 200;     //none
    let initial_power = 7.0;  //W
    let final_power   = 0.0;  //W
    let rate       = 1.0e7;   //W/s


    // Creating a mutable frames vector for the ramp
    let mut frames = vec![];

    // Appending the ramp powers for each frame to the vector, this in the form of a paired list (time, componant value)
    for i in 0..sim_length {
        frames.append(
            &mut vec![
                (
                    i as f64 * dt,
                    GaussianBeam {
                        intersection: Vector3::new(0.0, 0.0, 0.0),
                        e_radius:  e_radius,
                        // This is a exponetial ramp down of the power
                        power: ( initial_power - final_power ) * 2.0_f64.powf( -1.0 * rate * i as f64 * dt ) + final_power,
                        direction: Vector3::x(),
                        rayleigh_range: crate::laser::gaussian::calculate_rayleigh_range(&wavelength, &e_radius),
                        ellipticity: 0.0
                    }
                )
            ]
        )
    }

    let ramp = Ramp{
        prev: 0,
        keyframes: frames,
    };


    let gaussian_beam = GaussianBeam {
        intersection: Vector3::new(0.0, 0.0, 0.0),
        e_radius:  e_radius,
        power: 7.0,
        direction: Vector3::x(),
        rayleigh_range: crate::laser::gaussian::calculate_rayleigh_range(&wavelength, &e_radius),
        ellipticity: 0.0,
    };

    sim.world
        .create_entity()
        .with(gaussian_beam)
        .with(dipole::DipoleLight { wavelength })
        .with(laser::frame::Frame {
            x_vector: Vector3::y(),
            y_vector: Vector3::z(),
        })
        .with(ramp) // Adding the rmap to the gassian beam entity
        .build();

    // Create a single test atom
    sim.world
        .create_entity()
        .with(Atom)
        .with(Mass { value: 87.0 })
        .with(Force::new())
        .with(Position {
            pos: Vector3::new(
                50e-7,
                50e-7,
                50e-7,
            ),
        })
        .with(Velocity {
            vel: Vector3::new(
                0.0,
                0.0,
                0.0,
            ),
        })

        .with(lib::initiate::NewlyCreated)
        .build();

    // Define timestep
    sim.world.insert(Timestep { delta: dt});

    // Run the simulation for a number of steps.
    for _i in 0..sim_length {
        sim.step();
    }
    println!("Simulation completed in {} ms.", now.elapsed().as_millis());
}
