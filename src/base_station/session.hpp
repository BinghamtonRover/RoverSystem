#ifndef SESSION
#define SESSION

#include "../network/network.hpp"
#include "gui.hpp"

class Session {
private:
public:
    //Variables (Definitions)
    network::ModeMessage::Mode mode;
    gui::autonomy_info_struct autonomy_info;
    network::Feed r_feed;
    network::Feed bs_feed;
    gui::Font global_font;

    //Constructor & Destructor
    Session();
    ~Session();
};

#endif