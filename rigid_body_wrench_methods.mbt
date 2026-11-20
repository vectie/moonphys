///|
pub extend RigidBodyMotionState with @debug.Debug::{to_repr}

///|
pub extend RigidBodyMotionState with Eq::{not_equal, equal}

///|
pub extend RigidBodyMotionState with ToJson::{to_json}

///|
pub extend RigidBodyMotionState with @json.FromJson::{from_json}

///|
pub extend RigidBodyWrench with @debug.Debug::{to_repr}

///|
pub extend RigidBodyWrench with Eq::{not_equal, equal}

///|
pub extend RigidBodyWrench with ToJson::{to_json}

///|
pub extend RigidBodyWrench with @json.FromJson::{from_json}

///|
pub extend RigidBodyWrenchStep with @debug.Debug::{to_repr}

///|
pub extend RigidBodyWrenchStep with Eq::{not_equal, equal}

///|
pub extend RigidBodyWrenchStep with ToJson::{to_json}

///|
pub extend RigidBodyWrenchStep with @json.FromJson::{from_json}
