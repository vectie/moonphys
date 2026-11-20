///|
pub extend ContactProbe with @debug.Debug::{to_repr}

///|
pub extend ContactProbe with Eq::{not_equal, equal}

///|
pub extend ContactProbe with ToJson::{to_json}

///|
pub extend ContactProbe with @json.FromJson::{from_json}

///|
pub extend HeightfieldContactPatch with @debug.Debug::{to_repr}

///|
pub extend HeightfieldContactPatch with Eq::{not_equal, equal}

///|
pub extend HeightfieldContactPatch with ToJson::{to_json}

///|
pub extend HeightfieldContactPatch with @json.FromJson::{from_json}

///|
pub extend ContactPatchLoadSample with @debug.Debug::{to_repr}

///|
pub extend ContactPatchLoadSample with Eq::{not_equal, equal}

///|
pub extend ContactPatchLoadSample with ToJson::{to_json}

///|
pub extend ContactPatchLoadSample with @json.FromJson::{from_json}

///|
pub extend ContactPatchLoadAssessment with @debug.Debug::{to_repr}

///|
pub extend ContactPatchLoadAssessment with Eq::{not_equal, equal}

///|
pub extend ContactPatchLoadAssessment with ToJson::{to_json}

///|
pub extend ContactPatchLoadAssessment with @json.FromJson::{from_json}

///|
pub extend ContactPatchWrenchAssessment with @debug.Debug::{to_repr}

///|
pub extend ContactPatchWrenchAssessment with Eq::{not_equal, equal}

///|
pub extend ContactPatchWrenchAssessment with ToJson::{to_json}

///|
pub extend ContactPatchWrenchAssessment with @json.FromJson::{from_json}
