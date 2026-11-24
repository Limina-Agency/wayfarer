# Walking pace model

Status: built-in default and team calibration in `wayfarer_costing::pace` and
`wayfarer::pace_calibration`. No route reads it yet. The SAR reach work and the terrain
access legs (X3) take a `PaceModel`, and terrain heights arrive with format 4.

## Model

```text
duration = length / (flatSpeed * slope(grade) * trail(class)) * percentile
```

- `flatSpeedMmS`: moving speed on the flat, mm/s. Built in: 1,041 (3.75 km/h). It is a
  moving pace that includes slow stretches and stops under a minute, but not rests.
- `slopePermille`: 201 entries, per mille of the flat speed, for grade -100 % to +100 % in
  1 % steps, linear between steps, clamped outside. Built in: the E4 expected-pace fit
  (Campbell's Lorentz form, fitted by least squares on pace in s/m). It peaks at 1,008 at
  -3 % to -2 %, and gives 873 at +10 %, 696 at +20 %, 811 at -20 % and 545 at -40 %.
  Tobler gives 705, 497, 705 and 350 there. Beyond about +-58 % (30 degrees) it is extrapolated.
- `trailPermille`: paved 1,083, track 1,092, path 996, mountain path (`sac_scale=mountain_hiking`)
  845, alpine path (`demanding_mountain_hiking` and harder) 832, steps 922. Unknown ways use 1,000,
  the calibration mix. `PaceTrailClass::from_osm` classifies `highway` and `sac_scale`.
- `timePercentilesPermille`: P10 728, P50 982, P90 1,321, as multipliers on the expected time.
  These are the spread between walkers, measured per walk, not per segment.
- `source`: `default`, or `team` with the label, tracks, distance and moving time used.

Every evaluation is integer arithmetic (u128 for the time), so it gives the same result on
every target. `PaceModel::from_json` validates a calibration file (version, table length,
ranges, percentile order).

## Where the numbers come from

Lane TERRX experiment E4 (2026-11-24), project file `off-road-sar/exp/E4/README.md`:
2,945 km of walking from 1,625 OSM public GPS tracks in six areas of Małopolskie and
northern Slovakia. Heights come from GLO-30 on the 1" lattice. Trail versus off-trail is
decided against OSM ways. On 522 summer walks on trails, the model predicts moving time with
a median 0.98x the actual time and 15.2 % median error (15.6 % on held-out walks). Irmischer-Clarke
gives 15.9 %, Campbell 2022 P50 17.6 %, Tobler 19.5 %, Naismith-Langmuir 20.7 %, and the flat
4.8 km/h of today's OnFoot profile 42.4 %. The DEM barely matters: GLO-30 1", GLO-30 3" and SRTM
1" agree within 0.3 points. Off-trail cover factors are not part of the model: public traces
hold almost no real cross-country walking. They stay with X3's land-cover factors until a field trial.

## Team calibration

`calibrate_pace(tracks, base, elevation, options)` (`wayfarer pace calibrate` on the CLI):

1. Split at gaps over 120 s or jumps over 200 m. Drop pieces under 5 minutes, and pieces
   that are not walking: more than 5 % of their 60 s windows over 2.5 m/s, any window over
   4 m/s, or a median over 1.8 m/s.
2. Cut out breaks (every point within +-30 s inside 12 m). Smooth over 5 points. Cut 50 m segments.
3. Team factor = total moving time / the base model's expected time. The flat speed is
   divided by it.
4. Optionally reshape the slope curve: per 5 % grade bin from -40 % to +40 %, the bin's time
   ratio over the team factor, pulled toward 1 by `priorSegments` (100) pseudo-segments, as a
   speed multiplier interpolated between bin centres and normalised so that grade 0 stays at 1,000.
5. With five or more walks of at least 1 km, P10/P50/P90 become the spread of the team's own
   walks over the calibrated model.

Heights come from GPX `<ele>`, or from a caller-supplied elevation function (a DEM once
terrain files exist). Pieces with more than a fifth of their heights missing are dropped.
Trail types are not detected: `class` names what the tracks were walked on, and the ratios
between trail types stay those of the base model.

Check on real tracks: calibrating on the E4 Pieniny/Beskid Sądecki traces (971 km, 476 pieces)
gives a team factor of 0.934 and a calibrated factor of 0.995. Babia Góra (225 km, 125 pieces)
gives 1.048 and 0.997. The whole Pieniny set runs in 0.6 s.
