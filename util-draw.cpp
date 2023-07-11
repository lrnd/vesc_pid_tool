#include <gtk/gtk.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <map>
#include <ctype.h>
#include <string>
#include <list>
#include <vector>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <mutex>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cairo.h>
//#include "gnuplot-iostream.h"

using namespace std;
//#include <gtkmn.h>
//CAN msg IDs for VESC
#define CAN_PACKET_STATUS 9U
#define	CAN_PACKET_STATUS_4 16U
#define	CAN_PACKET_SET_RPM 3U
#define	CAN_PACKET_SET_CURRENT 1U
#define CAN_PACKET_SET_PID 253U
#define	CAN_PACKET_SET_POS_STEER 254U

//to store msg data
struct msg_data {
    double data;
    time_t time;
};

/********************GLOABALS*********************/
vector<string> vesc_ids;
GtkBuilder *builder;
int s;  //not sure if making this global is safe..
//to store msg data, globals for now
map<uint8_t, list<msg_data>> pos_set_map;
map<uint8_t, list<msg_data>> vel_set_map;
map<uint8_t, list<msg_data>> tq_set_map;

map<uint8_t, list<msg_data>> pos_map;
map<uint8_t, list<msg_data>> vel_map;
map<uint8_t, list<msg_data>> tq_map;
mutex map_mutex;

int current_vesc = -1;
int type = 0;
/*******************************************88*/

template<typename T>
void SwapEndian(T& val)
{
    union U {
        T val;
        uint8_t raw[sizeof(T)];
    } src, dst;

    src.val = val;
    reverse_copy(reinterpret_cast<uint8_t*>(&src.val),
                      reinterpret_cast<uint8_t*>(&src.val) + sizeof(T),
                      dst.raw);
    val = dst.val;
}

//called on init, to get vesc id data
void
set_combo_box_text_model(GtkComboBoxText *combo_box, string id = "NULL")
{
    GtkListStore *store;
    GtkTreeIter iter;

    //TODO loop through valid VESC IDs and add them to the combo box
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, "101");
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, "102");
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, "103");
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, "104");
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, "105");
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, "106");
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, "107");
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, "108");
    if (strcmp(id.c_str(), "NULL") != 0) {
        gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, id.c_str());
    }

    //used for normal combo box
    //gtk_combo_box_set_model(combo_box, GTK_TREE_MODEL(store));
    //g_object_unref(store);
}

void can_recv_thd()
{
    struct can_frame frame;
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    strcpy(ifr.ifr_name, "vcan0" );
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
    }

    auto start = chrono::high_resolution_clock::now();

    while (true) {
        printf("s %d\n", s);
        int nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("can raw socket read");
        }
        printf("canmsg");
        //assume is VESC msg
        uint32_t id = frame.can_id & (~CAN_EFF_FLAG);
        uint8_t ctr_id = id & 0xFF;
        uint8_t pkt_id = (id >> 8) & 0xFF;

        //assuming valid VESC ID
        if (id > 0 && id < 255 && frame.can_dlc == 8) {
            auto now = chrono::high_resolution_clock::now();
            auto count = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();

            //add ID to vesc_ids if not already there
            string vesc_id = to_string(id);
            if (find(vesc_ids.begin(), vesc_ids.end(), vesc_id) == vesc_ids.end()) {
                vesc_ids.push_back(vesc_id);
                set_combo_box_text_model(GTK_COMBO_BOX_TEXT(gtk_builder_get_object(builder, "ID")), vesc_id);
            }
            //get data
            uint8_t data[8];
            memcpy(data, frame.data, frame.can_dlc);
            switch (pkt_id) {
                //From VESC, actual state
                case CAN_PACKET_STATUS: {
                    //uint64_t* msg = (uint64_t*)data;
                    //SwapEndian(*msg);
                    int32_t vel = (int32_t)(*data);
                    SwapEndian(vel);
                    int16_t tq = (int16_t)data[4];  //TODO check this aye
                    SwapEndian(tq);

                    //add to map
                    map_mutex.lock();
                    vel_map[id].push_back({(double)vel/1120.0, (time_t)count});
                    if (vel_map[id].size() > 200) { vel_map[id].pop_front(); }

                    tq_map[id].push_back({(double)tq/1E3, (time_t)count});
                    if (tq_map[id].size() > 200) { tq_map[id].pop_front(); }
                    map_mutex.unlock();
                } break;
                case CAN_PACKET_STATUS_4: {
                    int16_t pos = (int16_t)data[6]; //lets also check here
                    SwapEndian(pos);
                    
                    map_mutex.lock();
                    pos_map[id].push_back({(double)pos/5E6, (time_t)count});
                    if (pos_map[id].size() > 200) { pos_map[id].pop_front(); }
                    map_mutex.unlock();

                } break;
                //From Controller to VESC, desired state
                case CAN_PACKET_SET_POS_STEER: {
                    int32_t pos = (int32_t)(*data);
                    SwapEndian(pos);
                    //add to map
                    map_mutex.lock();
                    pos_set_map[id].push_back({(double)pos/1E6, (time_t)count});
                    if (pos_set_map[id].size() > 200) { pos_set_map[id].pop_front(); }
                    map_mutex.unlock();
                }
                case CAN_PACKET_SET_RPM: {
                    int32_t vel = (int32_t)(*data);
                    SwapEndian(vel);
                    //add to map
                    map_mutex.lock();
                    vel_set_map[id].push_back({(double)vel/1E3, (time_t)count});
                    if (vel_set_map[id].size() > 200) { vel_set_map[id].pop_front(); }
                    map_mutex.unlock();
                }
                case CAN_PACKET_SET_CURRENT: {
                    int32_t tq = (int32_t)(*data);
                    SwapEndian(tq);
                    //add to map
                    map_mutex.lock();
                    tq_set_map[id].push_back({(double)tq, (time_t)count});
                    if (tq_set_map[id].size() > 200) { tq_set_map[id].pop_front(); }
                    map_mutex.unlock();
                }
            }
        }
    }
}

static void
send_pid_act (GtkWidget *widget, gpointer   user_data)
{
    //shoudn't need this... should be able to use write... TODO DEBUG..
    //struct sockaddr_can addr;
    //struct ifreq ifr;
    //strcpy(ifr.ifr_name, "vcan0");
    //ioctl(s, SIOCGIFINDEX, &ifr);
    //addr.can_ifindex = ifr.ifr_ifindex;
    //addr.can_family  = AF_CAN;
    //struct can_frame frame;
    //memset(&frame, 0, sizeof(frame));

    //TODO it seems i need to get a new socket for each thread... or I'm guessing the blocking read is blocking the write
    //TODO is there a better option here, seems to mean loopback is always true, which is fine for now
    struct can_frame frame;
    int sckt;
    struct sockaddr_can addr;
    struct ifreq ifr;

    sckt = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    strcpy(ifr.ifr_name, "vcan0" );
    ioctl(sckt, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    memset(&frame, 0, sizeof(frame));

    if (bind(sckt, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
    }
    //get current vesc_id from combo box
    GtkComboBoxText *combo_box = GTK_COMBO_BOX_TEXT(gtk_builder_get_object(builder, "ID"));
    gchar* vesc_id = gtk_combo_box_text_get_active_text(combo_box);
    uint8_t id = atoi(vesc_id);
    printf("id: %d\n", id);
    frame.can_id = ((id | ((uint32_t)CAN_PACKET_SET_PID << 8)) | CAN_EFF_FLAG);
    frame.can_dlc = 8;
    frame.__pad = 0;

    //pid msg, uint16_t P, uint16_t I, uint16_t D, spare...
    GtkEntry *p_entry = GTK_ENTRY(gtk_builder_get_object(builder, "P"));
    uint16_t p = atoi(gtk_entry_get_text(p_entry));
    printf("P: %d\n", p);
    SwapEndian(p);
    GtkEntry *i_entry = GTK_ENTRY(gtk_builder_get_object(builder, "I"));
    uint16_t i = atoi(gtk_entry_get_text(i_entry));
    printf("i: %d\n", i);
    SwapEndian(i);
    GtkEntry *d_entry = GTK_ENTRY(gtk_builder_get_object(builder, "D"));
    uint16_t d = atoi(gtk_entry_get_text(d_entry));
    printf("d: %d\n", d);
    SwapEndian(d);

    uint8_t data[8];
    memcpy(data, &p, 2);
    memcpy(data+2, &i, 2);
    memcpy(data+4, &d, 2);
    memcpy(frame.data, data, 8);
    printf("frame.data: %d\n", data[0]);


    //int nbytes = sendto(s, &frame, sizeof(struct can_frame),
    //                0, (struct sockaddr*)&addr, sizeof(addr));
    //TODO why is this write failing? but sendto works...
    int nbytes = write(sckt, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        printf("nbytes: %d\n", nbytes);
        perror("can raw socket write");
    }
    g_free(vesc_id);
}


#define M_TWOPI 6.283185307179586476925286766559
typedef unsigned long phase_t;
double maxphase = (double)((phase_t)0-(phase_t)1)+1.0;
double fs = 44100;
phase_t hz_to_delta( double hz )
{
    return maxphase*hz/fs+0.5;
}

float sample_phase( phase_t phase )
{
    return sin( phase/maxphase*M_TWOPI ) *1E3;
}

void test_data_thd()
{
    long i;
    phase_t delta, iphase = 0;
    long t = 0;

    while (true) {
        delta = hz_to_delta( 500.0 );
        for( i=0; i<fs; ++i ) {
            //printf( "%e\n", sample_phase( iphase += delta ) );
            pos_map[101].push_back({sample_phase(iphase += delta), t*1000});
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            t+=10;
        }

        delta = hz_to_delta( 1000.0 );
        for( i=0; i<fs; ++i ) {
            //printf( "%e\n", sample_phase( iphase += delta ) );
            pos_map[101].push_back({sample_phase(iphase += delta), t*1000});
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            t+=10;
        }
        t = 0;
    }
    return;

}

//run plot every 10 seconds, at 20hz control msgs that is about 200 data points
void plot_data_thd()
{
    //TODO add a sleep to allow for data to be recieved before we get plotting..
    sleep(3);
    bool init_plot;
    GtkComboBoxText *combo_box = GTK_COMBO_BOX_TEXT(gtk_builder_get_object(builder, "ID"));
    gchar* vesc_id = gtk_combo_box_text_get_active_text(combo_box);
    //GtkImage *image = GTK_IMAGE(gtk_builder_get_object(builder, "Graph"));
    //JUST AS A TEST GONNA ADD DUMMY DATA
//    for (int i = 0; i < 200; i++) {
//        pos_map[101].push_back({i*10.0, i});
//        pos_set_map[101].push_back({200.0,i});
//    }

    GtkWidget *area = GTK_WIDGET(gtk_builder_get_object (builder, "Graph"));

    while (true) {
        init_plot = true;
        char prev_id[10];
        vesc_id = gtk_combo_box_text_get_active_text(combo_box);
        mempcpy(prev_id, vesc_id, 10);

        while (isdigit(*vesc_id) && strcmp(vesc_id, prev_id) == 0) {
            //TODO maybe don't get this so fast...
            vesc_id = gtk_combo_box_text_get_active_text(combo_box);
            current_vesc = atoi(vesc_id);
            if (init_plot) {
                //TODO init plot
                init_plot = false;
                if (pos_map.find(atoi(vesc_id)) != pos_map.end()) {
                    type = 1;
                }
                else if (vel_map.find(atoi(vesc_id)) != vel_map.end()) {
                    type = 2;
                }
                else if (tq_map.find(atoi(vesc_id)) != tq_map.end()) {
                    type = 3;
                }
                else {
                    //TODO error
                    type = 0;
                    init_plot = true;
                }
            }

            //ofstream file("data.txt");
            //switch (type) {
            //    case 1: { //pos {}
            //        list<msg_data> pos = pos_map[atoi(vesc_id)];
            //        list<msg_data> set_pos = pos_set_map[atoi(vesc_id)];
            //        while (!pos.empty() || !set_pos.empty()) {
            //            if (!pos.empty()) { file << pos.front().time << " " << pos.front().data << " "; pos.pop_front();}
            //            if (!set_pos.empty()) { file << set_pos.front().time << " " << set_pos.front().data; set_pos.pop_front();}
            //            file << endl;
            //        }
            //    } break;
            //    case 2: //vel
            //        //todo write vel data to file
            //        break;
            //    case 3: //tq
            //        break;
            //}
            //file.close();
            //system("gnuplot -e \"set terminal png; set output 'image.png'; plot 'data.txt' using 1:2 title 'Response' with lines, 'data.txt' using 3:4 title 'input' with lines\"");
            //gtk_image_set_from_file(image, "/home/sean/pid_util/image.png");
            gtk_widget_queue_draw(area);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        sleep(1);
    }
    g_free(vesc_id);
}

gfloat normaliseData (double max, double min, double scale_max, double scale_min, double data) 
{
    return (gfloat)(scale_max-scale_min)*((data - min) / (max - min)) + scale_min;
}

gboolean
draw_callback (GtkWidget *widget, cairo_t *cr, gpointer data)
{
    GdkRectangle da;            /* GtkDrawingArea size */
    gdouble dx = 5.0, dy = 5.0; /* Pixels between each point */
    gdouble i, clip_x1 = 0.0, clip_y1 = 0.0, clip_x2 = 0.0, clip_y2 = 0.0;
    
    GdkWindow *window = gtk_widget_get_window(widget);

    /* Determine GtkDrawingArea dimensions */
    gdk_window_get_geometry (window,
            &da.x,
            &da.y,
            &da.width,
            &da.height);

    /* Draw on a black background */
    cairo_set_source_rgb (cr, 0.0, 0.0, 0.0);
    cairo_paint (cr);

    /* Change the transformation matrix */
    cairo_translate (cr, da.width / 2, da.height / 2);
    cairo_scale (cr, 100, -100);

    /* Determine the data points to calculate (ie. those in the clipping zone */
    cairo_device_to_user_distance (cr, &dx, &dy);
    cairo_clip_extents (cr, &clip_x1, &clip_y1, &clip_x2, &clip_y2);
    cairo_set_line_width (cr, dx);

    /* Draws x and y axis */
    gdouble x_origin = clip_x1 + dx*2;
    gdouble y_origin = clip_y1 - dy*2;
    cairo_set_source_rgb (cr, 0.0, 1.0, 0.0);
    cairo_move_to (cr, clip_x1, y_origin);
    cairo_line_to (cr, clip_x2, y_origin);
    cairo_move_to (cr, x_origin, clip_y1);
    cairo_line_to (cr, x_origin, clip_y2);
    cairo_stroke (cr);

    list<msg_data> input;
    list<msg_data> response;
    switch (type) {
        case 1: { //pos {}
            printf("pos case\n");
            input = pos_set_map[current_vesc];
            response = pos_map[current_vesc];
        } break;
        case 2: //vel
            input = vel_set_map[current_vesc];
            response = vel_map[current_vesc];
        case 3: //tq
            //TODO implement
            break;
    }

    for (auto &p : response) {
        printf("x: %f, y: %f\n", p.time/1000.0, p.data);
    }

    //min/max to normalise. using scale from response, should be valid for input too
    auto min_data_it = min_element(response.begin(), response.end(), [](const msg_data& a, const msg_data& b) { return a.data < b.data; });
    auto max_data_it = max_element(response.begin(), response.end(), [](const msg_data& a, const msg_data& b) { return a.data < b.data; });
    auto min_time_it = min_element(response.begin(), response.end(), [](const msg_data& a, const msg_data& b) { return a.time < b.time; });
    auto max_time_it = max_element(response.begin(), response.end(), [](const msg_data& a, const msg_data& b) { return a.time < b.time; });
    while (input.size() > 1) {
        cairo_move_to(cr, normaliseData(max_time_it->time/1000.0, min_time_it->time/1000.0, (double)clip_x2, (double)x_origin, (double)input.front().time/1000.0),
                        normaliseData(max_data_it->data, min_data_it->data, (double)clip_y2, (double)y_origin, (double)input.front().data));
        input.pop_front();
        cairo_line_to(cr, normaliseData(max_time_it->time/1000.0, min_time_it->time/1000.0, (double)clip_x2, (double)x_origin, (double)input.front().time/1000.0),
                        normaliseData(max_data_it->data, min_data_it->data, (double)clip_y2, (double)y_origin, (double)input.front().data));
        /* Draw the curve */
        cairo_set_source_rgba (cr, 1, 0.2, 0.2, 0.6);
        cairo_stroke (cr);
    }
    while (response.size() > 1) {
        cairo_move_to(cr, normaliseData(max_time_it->time/1000.0, min_time_it->time/1000.0, (double)clip_x2, (double)x_origin, (double)response.front().time/1000.0),
                        normaliseData(max_data_it->data, min_data_it->data, (double)clip_y2, (double)y_origin, (double)response.front().data));
        response.pop_front();
        cairo_line_to(cr, normaliseData(max_time_it->time/1000.0, min_time_it->time/1000.0, (double)clip_x2, (double)x_origin, (double)response.front().time/1000.0),
                        normaliseData(max_data_it->data, min_data_it->data, (double)clip_y2, (double)y_origin, (double)response.front().data));

        /* Draw the curve */
        cairo_set_source_rgba (cr, 0.2, 0.2, 1.0, 0.6);
        cairo_stroke (cr);
    }

//    /* Draw the curve */
//    cairo_set_source_rgba (cr, 1, 0.2, 0.2, 0.6);
//    cairo_stroke (cr);

    return FALSE;
}







int main (int   argc,
      char *argv[])
{
    //GtkBuilder *builder;
    //GObject *window;
    GObject *button;
    GError *error = NULL;

    gtk_init (&argc, &argv);
    
    //load data.txt with default data
    
    ofstream file("data.txt");
    file << 0 << " " << 0 << " " << 0 << " " << 0 << endl;
    file.close();

    /* Construct a GtkBuilder instance and load our UI description */
    builder = gtk_builder_new ();
    if (gtk_builder_add_from_file (builder, "util-window-draw.ui", &error) == 0)
    {
        g_printerr ("Error loading file: %s\n", error->message);
        g_clear_error (&error);
        return 1;
    }
        
    //gtk_window_set_default_size(GTK_WINDOW(gtk_builder_get_object(builder, "Util")), 640, 800);

    set_combo_box_text_model(GTK_COMBO_BOX_TEXT(gtk_builder_get_object(builder, "ID")));
    //spin up can thread
    std::thread can_recv(can_recv_thd);
    can_recv.detach();
    std::thread plot_data(plot_data_thd);
    plot_data.detach();
    std::thread test_thd(test_data_thd);
    test_thd.detach();


     button = gtk_builder_get_object (builder, "PID_load");
     g_signal_connect (button, "clicked", G_CALLBACK (send_pid_act), NULL);

    //start gnuplot to plot data by using s system call
    //system("gnuplot -e \"set terminal png; set output 'image.png'; plot 'data.txt' using 1:2 with lines\"");
    g_signal_connect (gtk_builder_get_object(builder, "Util"), "destroy", gtk_main_quit, NULL);

    //g_signal_connect (gtk_builder_get_object (builder, "Graph"), "expose_event", G_CALLBACK (on_expose_event), NULL); 

    GtkWidget *area = GTK_WIDGET(gtk_builder_get_object (builder, "Graph"));
    gtk_widget_set_size_request (area, 100, 100);
    gtk_widget_set_redraw_on_allocate (area, TRUE);
    g_signal_connect (G_OBJECT (area), "draw",
                    G_CALLBACK (draw_callback), NULL);

    // button = gtk_builder_get_object (builder, "quit");
    // g_signal_connect (button, "clicked", G_CALLBACK (gtk_main_quit), NULL);

    gtk_main ();

    return 0;
}
