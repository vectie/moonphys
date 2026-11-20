///|
pub extend RigidBodyJointFrameBody with @debug.Debug::{to_repr}

///|
pub extend RigidBodyJointFrameBody with Eq::{not_equal, equal}

///|
pub extend RigidBodyJointFrameBody with ToJson::{to_json}

///|
pub extend RigidBodyJointFrameBody with @json.FromJson::{from_json}

///|
pub extend RigidBodyHingeJointFrameAssessment with @debug.Debug::{to_repr}

///|
pub extend RigidBodyHingeJointFrameAssessment with Eq::{not_equal, equal}

///|
pub extend RigidBodyHingeJointFrameAssessment with ToJson::{to_json}

///|
pub extend RigidBodyHingeJointFrameAssessment with @json.FromJson::{from_json}
