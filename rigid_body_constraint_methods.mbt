///|
pub extend RigidBodyWorldDistanceConstraint with @debug.Debug::{to_repr}

///|
pub extend RigidBodyWorldDistanceConstraint with Eq::{not_equal, equal}

///|
pub extend RigidBodyWorldDistanceConstraint with ToJson::{to_json}

///|
pub extend RigidBodyWorldDistanceConstraint with @json.FromJson::{from_json}

///|
pub extend RigidBodyWorldConstraintResolution with @debug.Debug::{to_repr}

///|
pub extend RigidBodyWorldConstraintResolution with Eq::{not_equal, equal}

///|
pub extend RigidBodyWorldConstraintResolution with ToJson::{to_json}

///|
pub extend RigidBodyWorldConstraintResolution with @json.FromJson::{from_json}

///|
pub extend RigidBodyWorldConstraintFrame with @debug.Debug::{to_repr}

///|
pub extend RigidBodyWorldConstraintFrame with Eq::{not_equal, equal}

///|
pub extend RigidBodyWorldConstraintFrame with ToJson::{to_json}

///|
pub extend RigidBodyWorldConstraintFrame with @json.FromJson::{from_json}
