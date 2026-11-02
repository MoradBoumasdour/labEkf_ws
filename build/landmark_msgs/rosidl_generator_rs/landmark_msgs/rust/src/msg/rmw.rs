#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "landmark_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__landmark_msgs__msg__Landmark() -> *const std::ffi::c_void;
}

#[link(name = "landmark_msgs__rosidl_generator_c")]
extern "C" {
    fn landmark_msgs__msg__Landmark__init(msg: *mut Landmark) -> bool;
    fn landmark_msgs__msg__Landmark__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Landmark>, size: usize) -> bool;
    fn landmark_msgs__msg__Landmark__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Landmark>);
    fn landmark_msgs__msg__Landmark__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Landmark>, out_seq: *mut rosidl_runtime_rs::Sequence<Landmark>) -> bool;
}

// Corresponds to landmark_msgs__msg__Landmark
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Landmark {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub hamming: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goodness: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub decision_margin: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub range: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub bearing: f32,

}



impl Default for Landmark {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !landmark_msgs__msg__Landmark__init(&mut msg as *mut _) {
        panic!("Call to landmark_msgs__msg__Landmark__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Landmark {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { landmark_msgs__msg__Landmark__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { landmark_msgs__msg__Landmark__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { landmark_msgs__msg__Landmark__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Landmark {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Landmark where Self: Sized {
  const TYPE_NAME: &'static str = "landmark_msgs/msg/Landmark";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__landmark_msgs__msg__Landmark() }
  }
}


#[link(name = "landmark_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__landmark_msgs__msg__LandmarkArray() -> *const std::ffi::c_void;
}

#[link(name = "landmark_msgs__rosidl_generator_c")]
extern "C" {
    fn landmark_msgs__msg__LandmarkArray__init(msg: *mut LandmarkArray) -> bool;
    fn landmark_msgs__msg__LandmarkArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LandmarkArray>, size: usize) -> bool;
    fn landmark_msgs__msg__LandmarkArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LandmarkArray>);
    fn landmark_msgs__msg__LandmarkArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LandmarkArray>, out_seq: *mut rosidl_runtime_rs::Sequence<LandmarkArray>) -> bool;
}

// Corresponds to landmark_msgs__msg__LandmarkArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LandmarkArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub landmarks: rosidl_runtime_rs::Sequence<super::super::msg::rmw::Landmark>,

}



impl Default for LandmarkArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !landmark_msgs__msg__LandmarkArray__init(&mut msg as *mut _) {
        panic!("Call to landmark_msgs__msg__LandmarkArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LandmarkArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { landmark_msgs__msg__LandmarkArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { landmark_msgs__msg__LandmarkArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { landmark_msgs__msg__LandmarkArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LandmarkArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LandmarkArray where Self: Sized {
  const TYPE_NAME: &'static str = "landmark_msgs/msg/LandmarkArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__landmark_msgs__msg__LandmarkArray() }
  }
}


