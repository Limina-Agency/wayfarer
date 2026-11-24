//! Cooperative cancellation and deadlines for searches.
//!
//! A request carries a [`RouteCancellation`] in `RouteOptions::cancellation`:
//! any number of [`CancelToken`]s and an optional deadline. Every search loop
//! (routes in both directions, overlay patch searches, matrix cells and rows,
//! isochrones) checks it on its first visited edge and every
//! [`CHECK_INTERVAL`] after that, on
//! the same path as the visited-edge budget, and stops; the request then fails
//! with `RouteError::Cancelled` (a matrix fails each searchable cell with
//! `cancelled` or `deadline_exceeded`). Requests also check it before they
//! start and after their searches, so a token cancelled at any point before
//! the request returns is never ignored.

use std::time::{Duration, Instant};

pub use wayfarer_package::CancelToken;

use crate::types::RouteError;

/// Searches check for cancellation once every this many visited edges: about
/// tens of microseconds of search, and one relaxed load (plus a clock read
/// with a deadline) per check.
pub(crate) const CHECK_INTERVAL: usize = 1024;

/// When a request's searches stop early: once any of its tokens is cancelled
/// or its deadline passes. The default never stops. It is not part of the
/// request's JSON form; set it from code (the C ABI takes a token handle).
#[derive(Debug, Clone, Default)]
pub struct RouteCancellation {
    tokens: Vec<CancelToken>,
    deadline: Option<Instant>,
}

impl RouteCancellation {
    /// Also stops when `token` is cancelled.
    pub fn with_token(mut self, token: CancelToken) -> Self {
        self.tokens.push(token);
        self
    }

    /// Also stops at `deadline` (the earliest of several wins).
    pub fn with_deadline(mut self, deadline: Instant) -> Self {
        self.deadline = Some(self.deadline.map_or(deadline, |own| own.min(deadline)));
        self
    }

    /// Also stops `timeout` from now.
    pub fn with_timeout(self, timeout: Duration) -> Self {
        match Instant::now().checked_add(timeout) {
            Some(deadline) => self.with_deadline(deadline),
            None => self,
        }
    }

    pub fn deadline(&self) -> Option<Instant> {
        self.deadline
    }

    /// A token was cancelled or the deadline has passed.
    pub fn is_cancelled(&self) -> bool {
        self.tokens.iter().any(CancelToken::is_cancelled) || self.deadline_passed()
    }

    fn deadline_passed(&self) -> bool {
        self.deadline
            .is_some_and(|deadline| Instant::now() >= deadline)
    }

    /// `Err(RouteError::Cancelled)` once cancelled (see [`Self::is_cancelled`]).
    pub fn check(&self) -> Result<(), RouteError> {
        if self.tokens.iter().any(CancelToken::is_cancelled) {
            Err(RouteError::Cancelled {
                deadline_exceeded: false,
            })
        } else if self.deadline_passed() {
            Err(RouteError::Cancelled {
                deadline_exceeded: true,
            })
        } else {
            Ok(())
        }
    }

    /// The search loops' check after `visits` visited edges: cancelled, looked
    /// at on the first visit (so each of a request's many searches stops at
    /// once) and every [`CHECK_INTERVAL`] visits after it.
    #[inline]
    pub(crate) fn interrupted_at(&self, visits: usize) -> bool {
        visits % CHECK_INTERVAL == 1 && self.is_cancelled()
    }
}

/// Equal when they share the same tokens and deadline.
impl PartialEq for RouteCancellation {
    fn eq(&self, other: &Self) -> bool {
        self.deadline == other.deadline
            && self.tokens.len() == other.tokens.len()
            && self
                .tokens
                .iter()
                .zip(&other.tokens)
                .all(|(own, theirs)| own.same_as(theirs))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tokens_and_deadlines_stop_and_say_why() {
        let none = RouteCancellation::default();
        assert!(!none.is_cancelled());
        assert_eq!(none.check(), Ok(()));
        assert!(!none.interrupted_at(1));

        let token = CancelToken::new();
        let cancellation = RouteCancellation::default().with_token(token.clone());
        assert!(!cancellation.is_cancelled());
        token.clone().cancel();
        assert!(cancellation.is_cancelled());
        assert_eq!(
            cancellation.check(),
            Err(RouteError::Cancelled {
                deadline_exceeded: false
            })
        );
        // Checked on the first visit and every `CHECK_INTERVAL` after it.
        assert!(cancellation.interrupted_at(1));
        assert!(!cancellation.interrupted_at(CHECK_INTERVAL));
        assert!(cancellation.interrupted_at(2 * CHECK_INTERVAL + 1));

        let past = RouteCancellation::default().with_timeout(Duration::ZERO);
        assert_eq!(
            past.check(),
            Err(RouteError::Cancelled {
                deadline_exceeded: true
            })
        );
        let later = Instant::now() + Duration::from_secs(3600);
        let earliest = RouteCancellation::default()
            .with_deadline(later)
            .with_timeout(Duration::from_secs(60));
        assert!(earliest.deadline().unwrap() < later);
        assert!(!earliest.is_cancelled());
        assert!(
            RouteCancellation::default()
                .with_timeout(Duration::MAX)
                .deadline()
                .is_none()
        );

        assert_eq!(cancellation, cancellation.clone());
        assert_ne!(
            cancellation,
            RouteCancellation::default().with_token(CancelToken::new())
        );
    }

    mod searches {
        use std::collections::BTreeSet;
        use std::sync::OnceLock;

        use wayfarer_package::{DirectionalAccess, RoadType};
        use wayfarer_road_graph::{RoadGraphBuilder, RoutableWay, enhance_road_graph};

        use super::*;
        use crate::matrix::{MatrixMode, test_mode};
        use crate::test_support::PackageBuildOptions;
        use crate::{
            GeoPoint, LegRequest, MobilityProfile, RouteDynamicOverlay, RouteEdgeRange,
            RouteIsochroneContour, RouteIsochroneMetric, RouteIsochroneRequest, RouteMatrixRequest,
            RoutingEngine,
        };

        /// Rows and columns of the grid: large enough that every search kind
        /// runs for a while in a debug build.
        const SIDE: usize = 120;

        fn at(row: usize, col: usize) -> (f64, f64) {
            (50.0 + row as f64 * 0.0009, 20.0 + col as f64 * 0.0014)
        }

        /// A point in the middle of the street east of node (`row`, `col`).
        fn point(row: usize, col: usize) -> GeoPoint {
            let (lat, lng) = at(row, col);
            GeoPoint::new(lat, lng + 0.0007)
        }

        /// A residential grid without a CCH order (the searches run on their
        /// geometric bounds alone and explore widely).
        fn engine() -> &'static RoutingEngine {
            static ENGINE: OnceLock<RoutingEngine> = OnceLock::new();
            ENGINE.get_or_init(|| {
                let node = |(row, col): (usize, usize)| 1 + (row * SIDE + col) as i64;
                let mut builder = RoadGraphBuilder::new("cancellation-grid");
                let mut way_id = 0;
                let mut street = |a: (usize, usize), b: (usize, usize)| {
                    way_id += 1;
                    builder.push_way(RoutableWay {
                        osm_way_id: way_id,
                        points: vec![at(a.0, a.1), at(b.0, b.1)],
                        source_node_ids: vec![Some(node(a)), Some(node(b))],
                        road_type: RoadType::Residential,
                        directional_access: DirectionalAccess::Both,
                        highway: Some("residential".into()),
                        ..Default::default()
                    });
                };
                for row in 0..SIDE {
                    for col in 0..SIDE {
                        if col + 1 < SIDE {
                            street((row, col), (row, col + 1));
                        }
                        if row + 1 < SIDE {
                            street((row, col), (row + 1, col));
                        }
                    }
                }
                let mut graph = builder.finish();
                enhance_road_graph(&mut graph);
                let options = PackageBuildOptions {
                    profiles: vec![MobilityProfile::Wheeled, MobilityProfile::OnFoot],
                    // No goal direction (no CCH order): searches flood the
                    // grid, so that a cancel lands mid-search.
                    cch_order: false,
                    ..crate::test_support::test_options()
                };
                let (package, _) = crate::test_support::package(&graph, &options).unwrap();
                RoutingEngine::from_package(std::sync::Arc::new(package))
            })
        }

        #[derive(Debug, Clone, Copy)]
        enum Kind {
            Route,
            OverlayRoute,
            PairwiseMatrix,
            RowMatrix,
            Isochrone,
            OverlayIsochrone,
        }

        const KINDS: [Kind; 6] = [
            Kind::Route,
            Kind::OverlayRoute,
            Kind::PairwiseMatrix,
            Kind::RowMatrix,
            Kind::Isochrone,
            Kind::OverlayIsochrone,
        ];

        fn corner_route() -> LegRequest {
            LegRequest::new(
                point(0, 0),
                point(SIDE - 1, SIDE - 2),
                MobilityProfile::Wheeled,
            )
        }

        /// A partial block in the middle of the corner route: the request
        /// searches an overlay patch.
        fn overlay() -> RouteDynamicOverlay {
            static OVERLAY: OnceLock<RouteDynamicOverlay> = OnceLock::new();
            OVERLAY
                .get_or_init(|| {
                    let route = engine().route_leg(corner_route()).unwrap();
                    let edge_id = route.edge_ids[route.edge_ids.len() / 2];
                    RouteDynamicOverlay::from_blocked_ranges(&[RouteEdgeRange {
                        edge_id,
                        start_fraction: 0.4,
                        end_fraction: 0.6,
                    }])
                })
                .clone()
        }

        fn diagonal(count: usize) -> Vec<GeoPoint> {
            (0..count)
                .map(|index| {
                    let step = index * (SIDE - 2) / count.max(1);
                    point(step, SIDE - 2 - step)
                })
                .collect()
        }

        /// Runs one request of `kind`; a matrix whose cells were cancelled is
        /// reported as its cells' error.
        fn run(kind: Kind, cancellation: RouteCancellation) -> Result<(), RouteError> {
            let engine = engine();
            let matrix = |mode, sources: Vec<GeoPoint>, targets, profile| {
                let mut request = RouteMatrixRequest::new(sources, targets, profile);
                request.options.cancellation = cancellation.clone();
                test_mode::set(Some(mode));
                let result = engine.matrix(request);
                test_mode::set(None);
                let reasons = result
                    .cells
                    .iter()
                    .map(|cell| cell.failure_reason.as_deref())
                    .collect::<BTreeSet<_>>();
                match reasons.into_iter().collect::<Vec<_>>().as_slice() {
                    [Some("cancelled")] => Err(RouteError::Cancelled {
                        deadline_exceeded: false,
                    }),
                    [Some("deadline_exceeded")] => Err(RouteError::Cancelled {
                        deadline_exceeded: true,
                    }),
                    reasons => {
                        assert!(
                            !reasons
                                .iter()
                                .flatten()
                                .any(|reason| reason.contains("cancel")
                                    || reason.contains("deadline")),
                            "a cancelled matrix fails every cell alike: {reasons:?}"
                        );
                        Ok(())
                    }
                }
            };
            let isochrone = |overlay: Option<RouteDynamicOverlay>| {
                let mut request = RouteIsochroneRequest::new(
                    point(0, 0),
                    MobilityProfile::Wheeled,
                    vec![RouteIsochroneContour {
                        contour: 1e9,
                        metric: RouteIsochroneMetric::Cost,
                    }],
                );
                request.options.dynamic_overlay = overlay;
                request.options.cancellation = cancellation.clone();
                engine.isochrone(request).map(drop)
            };
            match kind {
                Kind::Route | Kind::OverlayRoute => {
                    let mut request = corner_route();
                    if matches!(kind, Kind::OverlayRoute) {
                        request.options.dynamic_overlay = Some(overlay());
                    }
                    request.options.cancellation = cancellation.clone();
                    engine
                        .route_leg(request)
                        .map(drop)
                        .map_err(|failure| failure.error)
                }
                Kind::PairwiseMatrix => matrix(
                    MatrixMode::Pairwise,
                    vec![point(0, 0), point(SIDE - 1, 0)],
                    vec![point(SIDE - 1, SIDE - 2), point(0, SIDE - 2)],
                    MobilityProfile::Wheeled,
                ),
                Kind::RowMatrix => matrix(
                    MatrixMode::Rows,
                    vec![point(0, 0), point(SIDE - 1, 0)],
                    diagonal(16),
                    MobilityProfile::Wheeled,
                ),
                Kind::Isochrone => isochrone(None),
                Kind::OverlayIsochrone => isochrone(Some(overlay())),
            }
        }

        #[test]
        fn every_search_kind_fails_at_once_when_cancelled_before_it_starts() {
            let token = CancelToken::new();
            token.cancel();
            let cancelled = RouteCancellation::default().with_token(token);
            let expired = RouteCancellation::default().with_deadline(Instant::now());
            let pending = RouteCancellation::default()
                .with_token(CancelToken::new())
                .with_timeout(Duration::from_secs(3600));
            for kind in KINDS {
                assert_eq!(
                    run(kind, cancelled.clone()),
                    Err(RouteError::Cancelled {
                        deadline_exceeded: false
                    }),
                    "{kind:?}"
                );
                assert_eq!(
                    run(kind, expired.clone()),
                    Err(RouteError::Cancelled {
                        deadline_exceeded: true
                    }),
                    "{kind:?}"
                );
                // A token never cancelled and a far deadline change nothing.
                assert_eq!(run(kind, pending.clone()), Ok(()), "{kind:?}");
            }
            // The route is the same with and without a pending cancellation.
            let mut request = corner_route();
            let plain = engine().route_leg(request.clone()).unwrap();
            request.options.cancellation = pending;
            let with_token = engine().route_leg(request).unwrap();
            assert_eq!(plain.edge_ids, with_token.edge_ids);
            assert_eq!(plain.cost, with_token.cost);
        }

        #[test]
        fn every_search_kind_stops_soon_after_a_cancel_from_another_thread() {
            for kind in KINDS {
                run(kind, RouteCancellation::default()).unwrap();
                let full = (0..2)
                    .map(|_| {
                        let started = Instant::now();
                        run(kind, RouteCancellation::default()).unwrap();
                        started.elapsed()
                    })
                    .min()
                    .unwrap();
                // Cancelled about a tenth of the way in, the request returns
                // well within half of its full time after the cancel. The time
                // counts from the cancel itself, not from the start: on a
                // shared machine the canceller can wake late, and that delay
                // says nothing about how soon the search stops. A cancel that
                // lands after the request finished is retried.
                let mut attempts = Vec::new();
                let stopped = (0..5).any(|_| {
                    let token = CancelToken::new();
                    let cancellation = RouteCancellation::default().with_token(token.clone());
                    let canceller = std::thread::spawn(move || {
                        std::thread::sleep(full / 10);
                        token.cancel();
                        Instant::now()
                    });
                    let started = Instant::now();
                    let outcome = run(kind, cancellation);
                    let finished = Instant::now();
                    let cancelled = canceller.join().unwrap();
                    let after_cancel = finished.saturating_duration_since(cancelled);
                    attempts.push((outcome.clone(), finished - started, after_cancel));
                    match outcome {
                        Err(RouteError::Cancelled {
                            deadline_exceeded: false,
                        }) => after_cancel < full / 2,
                        Ok(()) => false,
                        Err(error) => panic!("{kind:?}: {error}"),
                    }
                });
                assert!(
                    stopped,
                    "{kind:?} (full run {full:?}) did not stop early: {attempts:?} (outcome, elapsed, after the cancel)"
                );
            }
        }

        #[test]
        fn a_deadline_stops_a_running_route() {
            run(Kind::Route, RouteCancellation::default()).unwrap();
            let started = Instant::now();
            run(Kind::Route, RouteCancellation::default()).unwrap();
            let full = started.elapsed();
            let mut attempts = Vec::new();
            let stopped = (0..3).any(|_| {
                let started = Instant::now();
                let outcome = run(
                    Kind::Route,
                    RouteCancellation::default().with_timeout(full / 10),
                );
                let elapsed = started.elapsed();
                attempts.push((outcome.clone(), elapsed));
                outcome
                    == Err(RouteError::Cancelled {
                        deadline_exceeded: true,
                    })
                    && elapsed < full / 2
            });
            assert!(stopped, "full run {full:?}: {attempts:?}");
        }
    }
}
