//! Time spent at junctions: the delay a vehicle loses between two edges,
//! on top of the edges' own durations (which assume their speed throughout).
//!
//! Two parts, both per transition from one edge to the next at a node:
//!
//! - **Slowing down for the movement.** A turn is taken at a turning speed
//!   ([`turn_speed_mps`]: about 18 km/h for a right-angle turn, 13 km/h for a
//!   sharp one, walking pace for a U-turn); yielding (a minor approach, a
//!   give-way sign, a roundabout entry) limits it to [`YIELD_SPEED_MPS`] and a
//!   stop sign to zero. The time lost decelerating from the incoming edge's
//!   speed and accelerating to the outgoing edge's, at
//!   [`DECELERATION_MPS2`] and [`ACCELERATION_MPS2`], is
//!   `(v - u)^2 / (2 a v)` for each side. A short edge cannot be driven at its
//!   full speed between two such slow-downs, so its speed is capped by what
//!   the car reaches over its length; two turns through a short internal edge
//!   of a dual carriageway junction cost about what one turn does.
//! - **Waiting.** A yield costs [`YIELD_WAIT_S`] for a gap and a stop sign
//!   [`STOP_WAIT_S`]. Traffic signals cost the expected wait of an arrival at
//!   a random time plus, for the share of arrivals that meet red, the extra
//!   time of a full stop and start: [`SIGNAL_MAJOR`] on the major road of the
//!   junction, [`SIGNAL_MINOR`] on a minor approach, [`CROSSING_SIGNAL`] at a
//!   pedestrian crossing away from a junction (mostly green for cars).
//!
//! A straight-on movement with no control and no yield costs nothing. The
//! values are free-flow (off-peak) delays, at the low end of what traffic
//! engineering manuals give for uncongested junctions: a through movement on
//! a coordinated urban arterial loses about 6–8 s per signalised junction,
//! a minor approach 10–15 s.
//!
//! Walking profiles have no junction delay (their crossings are not
//! modelled); [`JunctionDelay::for_profile`] returns `None`. Emergency
//! vehicles wait [`EMERGENCY_WAIT_FACTOR`] of the time at controls.

use wayfarer_package::{MobilityProfile, TrafficControl};

/// Comfortable deceleration before a junction (m/s²).
pub const DECELERATION_MPS2: f64 = 2.0;
/// Acceleration out of a junction (m/s²).
pub const ACCELERATION_MPS2: f64 = 1.5;
/// Speed while yielding to crossing traffic (about 15 km/h).
pub const YIELD_SPEED_MPS: f64 = 4.0;
/// Expected wait for a gap when yielding.
pub const YIELD_WAIT_S: f64 = 2.0;
/// Wait at a stop sign after the full stop.
pub const STOP_WAIT_S: f64 = 2.5;
/// Manoeuvring time of a U-turn beyond its slow-down.
pub const U_TURN_MANOEUVRE_S: f64 = 3.0;
/// Share of the waits at controls an emergency vehicle spends.
pub const EMERGENCY_WAIT_FACTOR: f64 = 0.5;

/// A traffic signal's expected delay: `wait_s` for an arrival at a random
/// time, and the share of arrivals (`stop_share`) that stop and start again.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SignalDelay {
    pub wait_s: f64,
    pub stop_share: f64,
}

/// Signals met on the junction's major road.
pub const SIGNAL_MAJOR: SignalDelay = SignalDelay {
    wait_s: 4.0,
    stop_share: 0.35,
};
/// Signals met on a minor approach (shorter green).
pub const SIGNAL_MINOR: SignalDelay = SignalDelay {
    wait_s: 8.0,
    stop_share: 0.55,
};
/// Pedestrian crossing signals away from a junction (red on demand only).
pub const CROSSING_SIGNAL: SignalDelay = SignalDelay {
    wait_s: 1.0,
    stop_share: 0.15,
};

/// One transition between two edges at a node, as the delay model reads it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JunctionTransition {
    /// Heading change in degrees (0 straight on, 180 straight back); `None`
    /// when either heading is unknown.
    pub turn_deg: Option<u16>,
    /// Speed (m/s) and length (m) of the edge left and of the edge entered.
    pub previous_speed_mps: f64,
    pub previous_length_m: f64,
    pub next_speed_mps: f64,
    pub next_length_m: f64,
    /// The control met at the end of the edge left, and whether that end is a
    /// minor approach (see `RoadEdge::end_minor_approach`).
    pub control: TrafficControl,
    pub minor_approach: bool,
}

/// The junction delay model of one profile.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JunctionDelay {
    wait_factor: f64,
}

impl JunctionDelay {
    /// The model of `profile`; `None` for walking profiles.
    pub fn for_profile(profile: MobilityProfile) -> Option<Self> {
        match profile {
            MobilityProfile::Default | MobilityProfile::Wheeled | MobilityProfile::Tracked => {
                Some(Self { wait_factor: 1.0 })
            }
            MobilityProfile::Emergency => Some(Self {
                wait_factor: EMERGENCY_WAIT_FACTOR,
            }),
            MobilityProfile::OnFoot | MobilityProfile::Legged => None,
        }
    }

    /// Seconds lost at the junction (see the module documentation).
    pub fn delay_s(&self, t: &JunctionTransition) -> f64 {
        let turn_speed = t.turn_deg.map_or(f64::INFINITY, turn_speed_mps);
        let yields = t.minor_approach || t.control == TrafficControl::GiveWay;
        let control_speed = match t.control {
            TrafficControl::Stop => 0.0,
            TrafficControl::TrafficSignals | TrafficControl::CrossingSignals => f64::INFINITY,
            TrafficControl::GiveWay | TrafficControl::None if yields => YIELD_SPEED_MPS,
            TrafficControl::GiveWay | TrafficControl::None => f64::INFINITY,
        };
        let slowest = turn_speed.min(control_speed);
        let previous = |slowest: f64| {
            reachable_speed(
                t.previous_speed_mps,
                slowest,
                DECELERATION_MPS2,
                t.previous_length_m,
            )
        };
        let next = |slowest: f64| {
            reachable_speed(
                t.next_speed_mps,
                slowest,
                ACCELERATION_MPS2,
                t.next_length_m,
            )
        };
        // Time lost slowing to `u` and back (0 when no slowing is needed).
        let slow_down = |u: f64| {
            if !u.is_finite() {
                return 0.0;
            }
            lost_time_s(previous(u), u, DECELERATION_MPS2)
                + lost_time_s(next(u), u, ACCELERATION_MPS2)
        };
        let movement = slow_down(slowest);
        let mut wait = 0.0;
        let mut delay = movement;
        match t.control {
            TrafficControl::TrafficSignals | TrafficControl::CrossingSignals => {
                let signal = match (t.control, t.minor_approach) {
                    (TrafficControl::CrossingSignals, _) => CROSSING_SIGNAL,
                    (_, true) => SIGNAL_MINOR,
                    (_, false) => SIGNAL_MAJOR,
                };
                // Stopping at red costs the full stop beyond the movement's
                // own slow-down.
                let full_stop = slow_down(0.0);
                wait += signal.wait_s + signal.stop_share * (full_stop - movement).max(0.0);
            }
            TrafficControl::Stop => wait += STOP_WAIT_S,
            TrafficControl::GiveWay | TrafficControl::None if yields => wait += YIELD_WAIT_S,
            TrafficControl::GiveWay | TrafficControl::None => {}
        }
        if t.turn_deg.is_some_and(|deg| deg >= U_TURN_MIN_DEG) {
            delay += U_TURN_MANOEUVRE_S;
        }
        delay + wait * self.wait_factor
    }
}

/// Turns of at least this many degrees are U-turns (as for turn costs).
pub const U_TURN_MIN_DEG: u16 = 155;

/// The speed a turn of `turn_deg` degrees is taken at; infinite (no slowing)
/// below 35°, the turn costs' own threshold.
pub fn turn_speed_mps(turn_deg: u16) -> f64 {
    if turn_deg < 35 {
        f64::INFINITY
    } else if turn_deg < 100 {
        5.0
    } else if turn_deg < U_TURN_MIN_DEG {
        3.5
    } else {
        1.5
    }
}

/// An edge's speed where it meets the junction: its own speed, or less when
/// it is too short to reach it from (or brake from it to) `slowest` at
/// `rate`.
fn reachable_speed(speed_mps: f64, slowest: f64, rate: f64, length_m: f64) -> f64 {
    if !speed_mps.is_finite() || speed_mps <= 0.0 {
        return 0.0;
    }
    let reachable = (slowest * slowest + 2.0 * rate * length_m.max(0.0)).sqrt();
    speed_mps.min(reachable)
}

/// Time lost changing speed between `cruise` and `slow` at `rate` against
/// covering the same distance at `cruise`.
fn lost_time_s(cruise: f64, slow: f64, rate: f64) -> f64 {
    if cruise <= slow || cruise <= 0.0 {
        return 0.0;
    }
    let difference = cruise - slow;
    difference * difference / (2.0 * rate * cruise)
}

/// An edge's mean speed from its length and duration, 0 when unknown.
pub fn edge_speed_mps(length_m: f64, duration_s: f64) -> f64 {
    if duration_s > 0.0 && duration_s.is_finite() && length_m > 0.0 {
        length_m / duration_s
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CITY: f64 = 50.0 / 3.6;

    fn transition(turn_deg: u16) -> JunctionTransition {
        JunctionTransition {
            turn_deg: Some(turn_deg),
            previous_speed_mps: CITY,
            previous_length_m: 200.0,
            next_speed_mps: CITY,
            next_length_m: 200.0,
            control: TrafficControl::None,
            minor_approach: false,
        }
    }

    fn delay(t: JunctionTransition) -> f64 {
        JunctionDelay::for_profile(MobilityProfile::Wheeled)
            .unwrap()
            .delay_s(&t)
    }

    #[test]
    fn straight_on_through_an_uncontrolled_junction_is_free() {
        assert_eq!(delay(transition(10)), 0.0);
        assert_eq!(
            delay(JunctionTransition {
                turn_deg: None,
                ..transition(0)
            }),
            0.0
        );
    }

    #[test]
    fn turns_cost_their_slow_down() {
        let right_angle = delay(transition(90));
        assert!((3.0..3.6).contains(&right_angle), "{right_angle}");
        assert!(delay(transition(120)) > right_angle);
        let u_turn = delay(transition(180));
        assert!(u_turn > delay(transition(120)) + U_TURN_MANOEUVRE_S);
        // Slow residential streets lose little.
        let slow = delay(JunctionTransition {
            previous_speed_mps: 30.0 / 3.6,
            next_speed_mps: 30.0 / 3.6,
            ..transition(90)
        });
        assert!(slow < 1.0, "{slow}");
    }

    #[test]
    fn a_short_internal_edge_is_not_driven_at_full_speed() {
        let one_turn = delay(transition(90));
        let short = JunctionTransition {
            next_length_m: 12.0,
            ..transition(90)
        };
        let two_turns = delay(short)
            + delay(JunctionTransition {
                previous_length_m: 12.0,
                ..transition(90)
            });
        assert!(two_turns < 1.3 * one_turn, "{two_turns} vs {one_turn}");
    }

    #[test]
    fn controls_add_their_waits() {
        let straight = transition(0);
        let signals = delay(JunctionTransition {
            control: TrafficControl::TrafficSignals,
            ..straight
        });
        let minor_signals = delay(JunctionTransition {
            control: TrafficControl::TrafficSignals,
            minor_approach: true,
            ..straight
        });
        let crossing = delay(JunctionTransition {
            control: TrafficControl::CrossingSignals,
            ..straight
        });
        let yields = delay(JunctionTransition {
            minor_approach: true,
            ..straight
        });
        let stop = delay(JunctionTransition {
            control: TrafficControl::Stop,
            ..straight
        });
        assert!((6.0..8.5).contains(&signals), "{signals}");
        assert!(minor_signals > signals + 4.0, "{minor_signals}");
        assert!(crossing < 3.0, "{crossing}");
        assert!(yields > YIELD_WAIT_S && yields < stop, "{yields} {stop}");
        let emergency = JunctionDelay::for_profile(MobilityProfile::Emergency)
            .unwrap()
            .delay_s(&JunctionTransition {
                control: TrafficControl::TrafficSignals,
                ..straight
            });
        assert!(emergency < signals);
        assert!(JunctionDelay::for_profile(MobilityProfile::OnFoot).is_none());
    }
}
