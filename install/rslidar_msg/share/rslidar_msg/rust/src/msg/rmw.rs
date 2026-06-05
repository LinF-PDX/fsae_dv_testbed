#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "rslidar_msg__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rslidar_msg__msg__RslidarPacket() -> *const std::ffi::c_void;
}

#[link(name = "rslidar_msg__rosidl_generator_c")]
extern "C" {
    fn rslidar_msg__msg__RslidarPacket__init(msg: *mut RslidarPacket) -> bool;
    fn rslidar_msg__msg__RslidarPacket__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RslidarPacket>, size: usize) -> bool;
    fn rslidar_msg__msg__RslidarPacket__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RslidarPacket>);
    fn rslidar_msg__msg__RslidarPacket__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RslidarPacket>, out_seq: *mut rosidl_runtime_rs::Sequence<RslidarPacket>) -> bool;
}

// Corresponds to rslidar_msg__msg__RslidarPacket
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RslidarPacket {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_difop: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_frame_begin: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub data: rosidl_runtime_rs::Sequence<u8>,

}



impl Default for RslidarPacket {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rslidar_msg__msg__RslidarPacket__init(&mut msg as *mut _) {
        panic!("Call to rslidar_msg__msg__RslidarPacket__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RslidarPacket {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rslidar_msg__msg__RslidarPacket__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rslidar_msg__msg__RslidarPacket__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rslidar_msg__msg__RslidarPacket__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RslidarPacket {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RslidarPacket where Self: Sized {
  const TYPE_NAME: &'static str = "rslidar_msg/msg/RslidarPacket";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rslidar_msg__msg__RslidarPacket() }
  }
}


