///|
pub extend RigidBodyWorldReplayReview with @debug.Debug::{to_repr}

///|
pub extend RigidBodyWorldReplayReview with Eq::{not_equal, equal}

///|
pub extend RigidBodyWorldReplayReview with ToJson::{to_json}

///|
pub extend RigidBodyWorldReplayReview with @json.FromJson::{from_json}
