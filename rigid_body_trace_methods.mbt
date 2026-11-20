///|
pub extend RigidBodyHeightfieldFrame with @debug.Debug::{to_repr}

///|
pub extend RigidBodyHeightfieldFrame with Eq::{not_equal, equal}

///|
pub extend RigidBodyHeightfieldFrame with ToJson::{to_json}

///|
pub extend RigidBodyHeightfieldFrame with @json.FromJson::{from_json}

///|
pub extend RigidBodyHeightfieldTrace with @debug.Debug::{to_repr}

///|
pub extend RigidBodyHeightfieldTrace with Eq::{not_equal, equal}

///|
pub extend RigidBodyHeightfieldTrace with ToJson::{to_json}

///|
pub extend RigidBodyHeightfieldTrace with @json.FromJson::{from_json}
