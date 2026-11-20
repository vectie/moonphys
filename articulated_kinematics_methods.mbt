///|
pub extend ArticulatedLinkSpec with @debug.Debug::{to_repr}

///|
pub extend ArticulatedLinkSpec with Eq::{not_equal, equal}

///|
pub extend ArticulatedLinkSpec with ToJson::{to_json}

///|
pub extend ArticulatedLinkSpec with @json.FromJson::{from_json}

///|
pub extend ArticulatedJointPosition with @debug.Debug::{to_repr}

///|
pub extend ArticulatedJointPosition with Eq::{not_equal, equal}

///|
pub extend ArticulatedJointPosition with ToJson::{to_json}

///|
pub extend ArticulatedJointPosition with @json.FromJson::{from_json}

///|
pub extend ArticulatedLinkPose with @debug.Debug::{to_repr}

///|
pub extend ArticulatedLinkPose with Eq::{not_equal, equal}

///|
pub extend ArticulatedLinkPose with ToJson::{to_json}

///|
pub extend ArticulatedLinkPose with @json.FromJson::{from_json}

///|
pub extend ArticulatedPoseTree with @debug.Debug::{to_repr}

///|
pub extend ArticulatedPoseTree with Eq::{not_equal, equal}

///|
pub extend ArticulatedPoseTree with ToJson::{to_json}

///|
pub extend ArticulatedPoseTree with @json.FromJson::{from_json}
