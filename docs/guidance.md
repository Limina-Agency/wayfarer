# Guidance: instruction keys and their parameters

Every maneuver carries English text (`instruction`, `conciseInstruction`) and,
for hosts that render their own text, an `instructionKey` and string
`instructionParameters`. This table is the renderer contract: for each key,
the parameters a renderer may use. A parameter is present only when it has a
value. Other parameters in the map (`guidance_reason`, `significant_turn`,
`junction_*_count`, `connector_cost` and the like) are diagnostics, not part
of the contract.

## Keys

| Key | What happens | Renderer parameters |
|---|---|---|
| `route.start` | Depart | `road_name` |
| `route.continue` | Carry on where the road's name changes | `road_name`, `modifier` |
| `route.turn` | Turn | `modifier`, `road_name`, `toward`, `via` |
| `route.u_turn` | U-turn | `modifier` (`uturn`), `road_name` |
| `route.keep` | Keep to a side | `modifier`, `road_name`, `toward`, `via` |
| `route.fork` | Take a branch of a fork | `modifier`, `road_name`, `toward`, `via` |
| `route.merge` | Merge | `modifier`, `road_name`, `toward`, `via` |
| `route.ramp` | Take a ramp or an exit | `exit_number`, `road_name`, `toward`, `via` |
| `route.roundabout` | Enter a roundabout | `roundabout_exit_number`, `exit_road_name`, `toward`, `via` |
| `route.ferry` | Board a ferry (leaving it is a `route.continue` or a turn) | `road_name` (the ferry's name) |
| `route.hazard` | Hazard on the road ahead | `road_name` |
| `route.waypoint` | Arrive at a waypoint | `waypoint_index` |
| `route.destination` | Arrive | none |
| `route.ssvg_connector` | Follow an off-road connector | `dynamic_overlay_kind`, `connector_distance_m` |

## Parameters

| Parameter | Value |
|---|---|
| `road_name` | The name of the road entered |
| `modifier` | `straight`, `slight_left`, `left`, `sharp_left`, `slight_right`, `right`, `sharp_right` or `uturn`, from the turn angle: under 30° straight (slight towards its side for a keep, fork or merge), under 60° slight, under 120° plain, up to 135° sharp, beyond that `uturn` |
| `exit_number` | The exit's number: the link's `junction:ref`, or the `ref` of the motorway junction node it leaves from |
| `toward` | Where the signs point: `destination` and `destination:street`, `;`-separated, each once |
| `via` | The signed road number (`destination:ref`) |
| `roundabout_exit_number` | Which exit to take, counting the exits the profile may use |
| `exit_road_name` | The name (or ref) of the road the roundabout exit leads onto |
| `waypoint_index` | The waypoint's index among the request's locations |
| `road_type` | The class of the road entered (`motorway`, `primary`, `track`, ...) |
| `speed_limit_kph` | Its posted speed limit, rounded to km/h, when known |
| `toll`, `ferry`, `unpaved` | `true` when the road entered has tolls, is a ferry, or has an unpaved surface |

`sign_text`, `branch_text`, `destination`, `destination_ref`,
`destination_street` and `junction_ref` stay in the map for hosts that read
them. `sign_text` (and the "toward" in the English text) comes only from
destination signs, never from the road's own ref, and `branch_text` is a road
number, never an exit number.

## Concise text

`conciseInstruction` is what a maneuver leads onto, with the signs' guidance
and without the verb ("Zakopianka toward Kraków via A4" for "Turn right onto
Zakopianka toward Kraków via A4"), or the whole text when it leads onto
nothing named ("Arrive at destination").

## Navigation progress

`RouteNavigationInput::from_route` and `from_leg` fill
`shapeSegmentSpeedLimitsKph` (left out when no edge has a limit), and each
progress reports the active edge's `speedLimitKph`.
