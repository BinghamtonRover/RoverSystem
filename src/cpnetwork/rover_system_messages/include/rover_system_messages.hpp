#pragma once

#include <messages.hpp>
#include <video_control.pb.h>

namespace video_msg {
	DEFINE_MESSAGE_TYPE(Quality, video::Quality)
	DEFINE_MESSAGE_TYPE(Switch, video::Switch)
}

void register_messages() {
	msg::register_message_type<video_msg::Quality>();
	msg::register_message_type<video_msg::Switch>();
}