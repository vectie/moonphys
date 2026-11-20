///|
pub extend KinematicBodyState with @debug.Debug::{to_repr}

///|
pub extend KinematicBodyState with Eq::{not_equal, equal}

///|
pub extend KinematicBodyState with ToJson::{to_json}

///|
pub extend KinematicBodyState with @json.FromJson::{from_json}

///|
pub extend KinematicStep with @debug.Debug::{to_repr}

///|
pub extend KinematicStep with Eq::{not_equal, equal}

///|
pub extend KinematicStep with ToJson::{to_json}

///|
pub extend KinematicStep with @json.FromJson::{from_json}
