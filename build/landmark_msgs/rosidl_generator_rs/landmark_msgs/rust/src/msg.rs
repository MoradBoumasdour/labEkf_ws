#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to landmark_msgs__msg__Landmark

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Landmark::default())
  }
}

impl rosidl_runtime_rs::Message for Landmark {
  type RmwMsg = super::msg::rmw::Landmark;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id,
        hamming: msg.hamming,
        goodness: msg.goodness,
        decision_margin: msg.decision_margin,
        range: msg.range,
        bearing: msg.bearing,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      id: msg.id,
      hamming: msg.hamming,
      goodness: msg.goodness,
      decision_margin: msg.decision_margin,
      range: msg.range,
      bearing: msg.bearing,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id,
      hamming: msg.hamming,
      goodness: msg.goodness,
      decision_margin: msg.decision_margin,
      range: msg.range,
      bearing: msg.bearing,
    }
  }
}


// Corresponds to landmark_msgs__msg__LandmarkArray

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LandmarkArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub landmarks: Vec<super::msg::Landmark>,

}



impl Default for LandmarkArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::LandmarkArray::default())
  }
}

impl rosidl_runtime_rs::Message for LandmarkArray {
  type RmwMsg = super::msg::rmw::LandmarkArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        landmarks: msg.landmarks
          .into_iter()
          .map(|elem| super::msg::Landmark::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        landmarks: msg.landmarks
          .iter()
          .map(|elem| super::msg::Landmark::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      landmarks: msg.landmarks
          .into_iter()
          .map(super::msg::Landmark::from_rmw_message)
          .collect(),
    }
  }
}


