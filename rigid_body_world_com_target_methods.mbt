///|
pub extend RigidBodyWorldCenterOfMassTarget with @debug.Debug::{to_repr}

///|
pub extend RigidBodyWorldCenterOfMassTarget with Eq::{not_equal, equal}

///|
pub extend RigidBodyWorldCenterOfMassTarget with ToJson::{to_json}

///|
pub extend RigidBodyWorldCenterOfMassTarget with @json.FromJson::{from_json}
