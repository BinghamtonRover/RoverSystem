#include "waypoint.hpp"
#include "gui.hpp"
#include <string>
#include <iomanip> //std::setprecision
#include <sstream> 
#include <iostream>

namespace waypoint_menu {
	int selection = 0;
    int current_waypoints = 0;
    bool select = false;
    bool add_waypoint = false;
    int max_waypoints = 10;
    bool lat_box = true;
    unsigned int max_characters = 11;
    std::string lat_buffer = "";
    std::string lon_buffer = "";

	void do_menu(gui::Font * font, float x, float y){
		int text_height = 18;
		int vertical_spacing = 15; //spacing between text vertically
		int column_spacing = gui::text_width(font,"     ",text_height); //spacing = length of 5 spaces
		int title_height = 25;
        int sub_title_height = 20;
        int footer_text_height = 13;
		const char * title = "Waypoint Menu";
        const char * waypoint_title = "Waypoints";
        const char * add_waypoint_title = "Add Waypoint";
        const char * initial_text = "Press s to select a waypoint | Press a to add a waypoint | Press Esc to exit menu";
        const char * select_waypoint_text = "Select a waypoint using the UP and DOWN arrows | Press Enter to delete | Esc to quit";
        const char * add_waypoint_text = "Press TAB to change boxes | Press Enter to confirm | Esc to quit";
		int title_width = gui::text_width(font,title,title_height);
        int waypoint_title_width = gui::text_width(font,waypoint_title,sub_title_height);
        int add_waypoint_title_width = gui::text_width(font,add_waypoint_title,sub_title_height);
        int add_waypoint_area = 100;
        int top_padding = 20;
	    int bottom_padding = top_padding + 30; // +30 for help on the bottom
	    int menu_height = top_padding + title_height + vertical_spacing + sub_title_height + (vertical_spacing * max_waypoints) + (text_height * max_waypoints) + sub_title_height + add_waypoint_area + footer_text_height + bottom_padding;
	    int left_padding = 70;
        int left_padding_in_area = 15;
	    int right_padding = left_padding;
    	int menu_width = (menu_height - top_padding - bottom_padding) + left_padding + right_padding;
	    gui::Layout menu_layout;
	    x = x - menu_width/2;
	    y = y - menu_height/2;
	    menu_layout.advance_x(x);
    	menu_layout.advance_y(y);
    	menu_layout.push();

    	gui::do_solid_rect(&menu_layout, menu_width, menu_height, 0.2, 0.2f, 0.2f);
    	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    	gui::draw_text(font, title, x + (menu_width / 2) - (title_width / 2), y + top_padding, title_height); //title

        gui::draw_text( font,
                        waypoint_title,
                        x + menu_width/2 - waypoint_title_width/2,
                        y + top_padding + title_height + vertical_spacing,
                        sub_title_height
        );
        gui::draw_text( font,
                        add_waypoint_title,
                        x + menu_width/2 - add_waypoint_title_width/2,
                        y + menu_height- bottom_padding - add_waypoint_area,
                        sub_title_height
        );

    	int last_index = waypoint::cur_waypoints.size() - 1;
        int waypoints_displayed = 0;
    	for(int cur_index = last_index; last_index - cur_index < max_waypoints && cur_index >= 0; cur_index--){
    		float waypoint_lat = waypoint::cur_waypoints[cur_index].latitude;
    		float waypoint_lon = waypoint::cur_waypoints[cur_index].longitude;
            char lat_fixed[20];
            char lon_fixed[20];
            snprintf(lat_fixed,20,"%.7f",waypoint_lat);
            snprintf(lon_fixed,20,"%.7f",waypoint_lon);
            std::string left_paren("(");
            std::string right_paren(")");
            std::string comma(",");
            //const char * location = (left_paren + lat_fixed + comma + lon_fixed + right_paren).c_str();
            std::string location_str = left_paren + lat_fixed + comma + lon_fixed + right_paren;
            const char * location = location_str.c_str();
            int location_width = gui::text_width(font,location,text_height);
    		int waypoint_number = last_index - cur_index;
    		const char * waypoint_number_str = std::to_string(waypoint_number).c_str();
    		int waypoint_number_width = gui::text_width(font,waypoint_number_str,text_height);
            waypoints_displayed++;
    		gui::draw_text( font,
    						waypoint_number_str,
    						x + left_padding + left_padding_in_area,
    						y + top_padding + title_height + vertical_spacing + sub_title_height + (text_height * (waypoint_number + 1)) + (vertical_spacing * waypoint_number),
    						text_height
    		);
    		gui::draw_text( font,
    						location,
    						x + left_padding + left_padding_in_area + column_spacing,
    						y + top_padding + title_height + vertical_spacing + sub_title_height + (text_height * (waypoint_number + 1)) + (vertical_spacing * waypoint_number),
    						text_height
    		);
    	}
        //Updates global counter of displayed waypoints on the menu 
        if (waypoints_displayed != current_waypoints){
            current_waypoints = waypoints_displayed;
        }
        if (select && waypoint::cur_waypoints.size() > 0){
            glColor4f(0.0f, 0.0f, 1.0f, 0.5f);
            gui::fill_rectangle(x + left_padding, y + top_padding + title_height + (vertical_spacing * 2) + sub_title_height + (text_height * selection) + (vertical_spacing * selection), menu_width - left_padding - right_padding, text_height + vertical_spacing);
            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        }
        int box_width = gui::text_width(font,"       ",text_height + 20); //width = length of 7 spaces
        const char * lat = "Latitude: ";
        const char * lon = "Longitude: ";
        int lat_width = gui::text_width(font,lat,text_height);
        int lon_width = gui::text_width(font,lon,text_height);

        //Make "add waypoint" boxes
        gui::draw_text(font,lat,x + left_padding, y + menu_height - bottom_padding - add_waypoint_area + add_waypoint_area/2 - text_height,text_height);
        gui::draw_text(font,lon,x + left_padding + lat_width + box_width + column_spacing, y + menu_height - bottom_padding - add_waypoint_area + add_waypoint_area/2 - text_height,text_height);
        if (add_waypoint)
            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        else
            glColor4f(0.4f,0.4f,0.4f,0.9f);

        gui::fill_rectangle(x + left_padding + lat_width,y + menu_height - bottom_padding - add_waypoint_area + add_waypoint_area/2 - text_height,box_width,text_height);
        gui::fill_rectangle(x + left_padding + lat_width + box_width + column_spacing + lon_width,y + menu_height - bottom_padding - add_waypoint_area + add_waypoint_area/2 - text_height,box_width,text_height);
        //Deal with text
        int char_width = gui::text_width(font,"  ",text_height);
        if (add_waypoint){
            // Draw the cursor.
            glColor4f(0.0f, 0.0f, 1.0f, 0.5f);
            glLineWidth(3.0f);
            glBegin(GL_LINES);
            if (lat_box){
                glVertex2f(x + left_padding + lat_width + (lat_buffer.size() * (char_width + char_width/6)),y + menu_height - bottom_padding - add_waypoint_area + add_waypoint_area/2 - text_height);
                glVertex2f(x + left_padding + lat_width + (lat_buffer.size() * (char_width + char_width/6)),y + menu_height - bottom_padding - add_waypoint_area + add_waypoint_area/2);
            }
            else {
                glVertex2f(x + left_padding + lat_width + box_width + column_spacing + lon_width + (lon_buffer.size() * (char_width + char_width/6)),y + menu_height - bottom_padding - add_waypoint_area + add_waypoint_area/2 - text_height);
                glVertex2f(x + left_padding + lat_width + box_width + column_spacing + lon_width + (lon_buffer.size() * (char_width + char_width/6)),y + menu_height - bottom_padding - add_waypoint_area + add_waypoint_area/2);
            }
            glEnd();
            //Draw the text.
            glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
            gui::draw_text(font,lat_buffer.c_str(),x + left_padding + lat_width,y + menu_height - bottom_padding - add_waypoint_area + add_waypoint_area/2 - text_height,text_height);
            gui::draw_text(font,lon_buffer.c_str(),x + left_padding + lat_width + box_width + column_spacing + lon_width,y + menu_height - bottom_padding - add_waypoint_area + add_waypoint_area/2 - text_height,text_height);
        }

		glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
	    glLineWidth(3.5f);
	    glBegin(GL_LINES);
        // Left Margin Line
        glVertex2f(x + left_padding, y + top_padding + title_height + vertical_spacing);
        glVertex2f(x + left_padding, y + menu_height - bottom_padding);
        // Right Margin Line
        glVertex2f(x + menu_width - right_padding, y + top_padding + title_height + vertical_spacing);
        glVertex2f(x + menu_width - right_padding, y + menu_height - bottom_padding);
        //Beginning of Waypoint List Area
        glVertex2f(x + left_padding, y + top_padding + title_height + vertical_spacing);
        glVertex2f(x + menu_width - right_padding,y + top_padding + title_height + vertical_spacing);
        //Add Waypoint Area
        glVertex2f(x + left_padding, y + menu_height - bottom_padding - add_waypoint_area);
        glVertex2f(x + menu_width - right_padding, y + menu_height - bottom_padding - add_waypoint_area);
        glVertex2f(x + left_padding, y + menu_height - bottom_padding);
        glVertex2f(x + menu_width - right_padding, y + menu_height - bottom_padding);
        //Highlight Latitude or Longitude boxes
        if (lat_box && add_waypoint){
            glVertex2f(x + left_padding, y + menu_height - bottom_padding - add_waypoint_area + add_waypoint_area/2 + vertical_spacing/2);
            glVertex2f(x + left_padding + lat_width, y + menu_height - bottom_padding - add_waypoint_area + add_waypoint_area/2 + vertical_spacing/2);
        }
        else if (!lat_box && add_waypoint) {
            glVertex2f(x + left_padding + lat_width + box_width + column_spacing, y + menu_height - bottom_padding - add_waypoint_area + add_waypoint_area/2  + vertical_spacing/2);
            glVertex2f(x + left_padding + lat_width + box_width + column_spacing + lon_width, y + menu_height - bottom_padding - add_waypoint_area + add_waypoint_area/2 + vertical_spacing/2);
        }
		glEnd();
        //Footer Text
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        if (gui::state.input_state == gui::InputState::WAYPOINT_MENU)
            gui::draw_text(font,initial_text,x,y + menu_height - footer_text_height - 3,footer_text_height);
        else if (gui::state.input_state == gui::InputState::WAYPOINT_MENU_SELECT)
            gui::draw_text(font,select_waypoint_text,x,y + menu_height - footer_text_height - 3,footer_text_height);
        else if (gui::state.input_state == gui::InputState::WAYPOINT_MENU_ADD)
            gui::draw_text(font,add_waypoint_text,x,y + menu_height - footer_text_height - 3,footer_text_height);
    	menu_layout.pop();
	}
}//namespace waypoint_menu