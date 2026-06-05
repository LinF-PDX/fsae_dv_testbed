#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to rslidar_msg__msg__RslidarPacket

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RslidarPacket {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_difop: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_frame_begin: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub data: Vec<u8>,

}



impl Default for RslidarPacket {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RslidarPacket::default())
  }
}

impl rosidl_runtime_rs::Message for RslidarPacket {
  type RmwMsg = super::msg::rmw::RslidarPacket;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        is_difop: msg.is_difop,
        is_frame_begin: msg.is_frame_begin,
        data: msg.data.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      is_difop: msg.is_difop,
      is_frame_begin: msg.is_frame_begin,
        data: msg.data.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      is_difop: msg.is_difop,
      is_frame_begin: msg.is_frame_begin,
      data: msg.data
          .into_iter()
          .collect(),
    }
  }
}


