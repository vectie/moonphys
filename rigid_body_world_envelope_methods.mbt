///|
pub extend RigidBodyWorldBodyEnvelope with @debug.Debug::{to_repr}

///|
pub extend RigidBodyWorldBodyEnvelope with Eq::{not_equal, equal}

///|
pub extend RigidBodyWorldBodyEnvelope with ToJson::{to_json}

///|
pub extend RigidBodyWorldBodyEnvelope with @json.FromJson::{from_json}

///|
pub extend RigidBodyWorldTraceEnvelope with @debug.Debug::{to_repr}

///|
pub extend RigidBodyWorldTraceEnvelope with Eq::{not_equal, equal}

///|
pub extend RigidBodyWorldTraceEnvelope with ToJson::{to_json}

///|
pub extend RigidBodyWorldTraceEnvelope with @json.FromJson::{from_json}
