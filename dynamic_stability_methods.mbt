///|
pub extend CapturePointAssessment with @debug.Debug::{to_repr}

///|
pub extend CapturePointAssessment with Eq::{not_equal, equal}

///|
pub extend CapturePointAssessment with ToJson::{to_json}

///|
pub extend CapturePointAssessment with @json.FromJson::{from_json}
