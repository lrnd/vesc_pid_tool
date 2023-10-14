#include <gtk/gtk.h>
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
//#include "gnuplot-iostream.h"

using namespace std;
//#include <gtkmn.h>
//CAN msg IDs for VESC
typedef enum {
    CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
	CAN_PACKET_FILL_RX_BUFFER,
	CAN_PACKET_FILL_RX_BUFFER_LONG,
	CAN_PACKET_PROCESS_RX_BUFFER,
	CAN_PACKET_PROCESS_SHORT_BUFFER,
	CAN_PACKET_STATUS,
	CAN_PACKET_SET_CURRENT_REL,
	CAN_PACKET_SET_CURRENT_BRAKE_REL,
	CAN_PACKET_SET_CURRENT_HANDBRAKE,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
	CAN_PACKET_STATUS_2,
	CAN_PACKET_STATUS_3,
	CAN_PACKET_STATUS_4,
	CAN_PACKET_PING,
	CAN_PACKET_PONG,
	CAN_PACKET_DETECT_APPLY_ALL_FOC,
	CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
	CAN_PACKET_CONF_CURRENT_LIMITS,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
	CAN_PACKET_CONF_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_FOC_ERPMS,
	CAN_PACKET_CONF_STORE_FOC_ERPMS,
	CAN_PACKET_STATUS_5,
	CAN_PACKET_POLL_TS5700N8501_STATUS,
	CAN_PACKET_CONF_BATTERY_CUT,
	CAN_PACKET_CONF_STORE_BATTERY_CUT,
	CAN_PACKET_SHUTDOWN,
	CAN_PACKET_IO_BOARD_ADC_1_TO_4,
	CAN_PACKET_IO_BOARD_ADC_5_TO_8,
	CAN_PACKET_IO_BOARD_ADC_9_TO_12,
	CAN_PACKET_IO_BOARD_DIGITAL_IN,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM,
	CAN_PACKET_BMS_V_TOT,
	CAN_PACKET_BMS_I,
	CAN_PACKET_BMS_AH_WH,
	CAN_PACKET_BMS_V_CELL,
	CAN_PACKET_BMS_BAL,
	CAN_PACKET_BMS_TEMPS,
	CAN_PACKET_BMS_HUM,
	CAN_PACKET_BMS_SOC_SOH_TEMP_STAT,
	CAN_PACKET_PSW_STAT,
	CAN_PACKET_PSW_SWITCH,
	CAN_PACKET_BMS_HW_DATA_1,
	CAN_PACKET_BMS_HW_DATA_2,
	CAN_PACKET_BMS_HW_DATA_3,
	CAN_PACKET_BMS_HW_DATA_4,
	CAN_PACKET_BMS_HW_DATA_5,
	CAN_PACKET_BMS_AH_WH_CHG_TOTAL,
	CAN_PACKET_BMS_AH_WH_DIS_TOTAL,
	CAN_PACKET_UPDATE_PID_POS_OFFSET,
	CAN_PACKET_POLL_ROTOR_POS,
	CAN_PACKET_NOTIFY_BOOT,
	CAN_PACKET_STATUS_6,
	CAN_PACKET_GNSS_TIME,
	CAN_PACKET_GNSS_LAT,
	CAN_PACKET_GNSS_LON,
	CAN_PACKET_GNSS_ALT_SPEED_HDOP,
    CAN_PACKET_SET_PID = 253U,
	CAN_PACKET_SET_POS_STEER = 254U,
	CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
} CAN_PACKET_ID;

//to store msg data
struct msg_data {
    double data;
    time_t time;
};

/********************GLOABALS*********************/
vector<string> vesc_ids;
bool send_pid = false;
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
set_combo_box_text_model(GtkComboBoxText *combo_box)
{
    GtkListStore *store;
    GtkTreeIter iter;

    //store = gtk_list_store_new(1, G_TYPE_STRING);

    //TODO loop through valid VESC IDs and add them to the combo box
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, "101");
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, "102");
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, "103");
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, "104");
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, "105");
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, "106");
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, "107");
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_box), NULL, "108");

    //used for normal combo box
    //gtk_combo_box_set_model(combo_box, GTK_TREE_MODEL(store));
    //g_object_unref(store);
}

void manage_map_size() {
    map_mutex.lock();
    for (auto m : pos_map) {
        if (m.second.size() > 200) {
            m.second.pop_front();
        }
    }
    for (auto m : vel_map) {
        if (m.second.size() > 200) {
            m.second.pop_front();
        }
    }
    for (auto m : tq_map) {
        if (m.second.size() > 200) {
            m.second.pop_front();
        }
    }
    map_mutex.unlock();
}

void can_recv_thd()
{
    //int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    //TODO make this a input..
    const char *ifname = "vcan0";

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return;
    }

    auto start = chrono::high_resolution_clock::now();

    while (true) {
        printf("s %d\n", s);
        int nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("can raw socket read");
        }
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
                set_combo_box_text_model(GTK_COMBO_BOX_TEXT(gtk_builder_get_object(builder, "ID")));
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
                    tq_map[id].push_back({(double)tq/1E3, (time_t)count});
                    map_mutex.unlock();
                } break;
                case CAN_PACKET_STATUS_4: {
                    int16_t pos = (int16_t)data[6]; //lets also check here
                    SwapEndian(pos);
                    
                    map_mutex.lock();
                    pos_map[id].push_back({(double)pos/5E6, (time_t)count});
                    map_mutex.unlock();

                } break;
                //From Controller to VESC, desired state
                case CAN_PACKET_SET_POS_STEER: {
                    int32_t pos = (int32_t)(*data);
                    SwapEndian(pos);
                    //add to map
                    map_mutex.lock();
                    pos_set_map[id].push_back({(double)pos/1E6, (time_t)count});
                    map_mutex.unlock();
                }
                case CAN_PACKET_SET_RPM: {
                    int32_t vel = (int32_t)(*data);
                    SwapEndian(vel);
                    //add to map
                    map_mutex.lock();
                    vel_set_map[id].push_back({(double)vel/1E3, (time_t)count});
                    map_mutex.unlock();
                }
                case CAN_PACKET_SET_CURRENT: {
                    int32_t tq = (int32_t)(*data);
                    SwapEndian(tq);
                    //add to map
                    map_mutex.lock();
                    tq_set_map[id].push_back({(double)tq, (time_t)count});
                    map_mutex.unlock();
                }
            }
            //manage size of maps
            manage_map_size();
        }
    }
}

//Won't send if not recieving can... oh well
static void
send_pid_act (GtkWidget *widget, gpointer   user_data)
{
    //shoudn't need this... should be able to use write... TODO DEBUG..
    struct sockaddr_can addr;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "vcan0");
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_ifindex = ifr.ifr_ifindex;
    addr.can_family  = AF_CAN;
    struct can_frame frame;


    memset(&frame, 0, sizeof(frame));
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
    data[0] = p & 0xFF;
    data[2] = i & 0xFF;
    data[4] = d & 0xFF;
    //memcpy(data, &p, 2);
    //memcpy(data+2, &i, 2);
    //memcpy(data+4, &d, 2);
    memcpy(frame.data, data, 8);
    printf("frame.data: %d\n", data[0]);


    int nbytes = sendto(s, &frame, sizeof(struct can_frame),
                    0, (struct sockaddr*)&addr, sizeof(addr));
    //TODO why is this write failing? but sendto works...
    //int nbytes = write(s, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        printf("nbytes: %d\n", nbytes);
        perror("can raw socket write");
    }
    g_free(vesc_id);
    send_pid = false;
}

//run plot every 10 seconds, at 20hz control msgs that is about 200 data points
void plot_data_thd()
{
    //TODO add a sleep to allow for data to be recieved before we get plotting..
    sleep(3);
    bool init_plot;
    int type;
    GtkComboBoxText *combo_box = GTK_COMBO_BOX_TEXT(gtk_builder_get_object(builder, "ID"));
    gchar* vesc_id = gtk_combo_box_text_get_active_text(combo_box);
    GtkImage *image = GTK_IMAGE(gtk_builder_get_object(builder, "Graph"));
    //JUST AS A TEST GONNA ADD DUMMY DATA
    for (int i = 0; i < 200; i++) {
        pos_map[101].push_back({i*10.0, i});
        pos_set_map[101].push_back({200.0,i});
    }
    while (true) {
        init_plot = true;
        char prev_id[10];
        vesc_id = gtk_combo_box_text_get_active_text(combo_box);
        mempcpy(prev_id, vesc_id, 10);
        if (!isdigit(*vesc_id)) {
            printf("not digit\n");
        }
        if (strcmp(vesc_id, prev_id) != 0) {
            printf("not same\n");
        }
        while (isdigit(*vesc_id) && strcmp(vesc_id, prev_id) == 0) {
            //TODO maybe don't get this so fast...
            vesc_id = gtk_combo_box_text_get_active_text(combo_box);
            if (init_plot) {
                //TODO init plot
                init_plot = false;
                if (pos_map.find(atoi(vesc_id)) != pos_map.end()) {
                    //TODO plot
                    type = 1;
                }
                else if (vel_map.find(atoi(vesc_id)) != vel_map.end()) {
                    //TODO plot
                    type = 2;
                }
                else if (tq_map.find(atoi(vesc_id)) != tq_map.end()) {
                    //TODO plot
                    type = 3;
                }
                else {
                    //TODO error
                    type = 0;
                    init_plot = true;
                }
            }

            ofstream file("data.txt");
            switch (type) {
                case 1: { //pos {}
                    list<msg_data> pos = pos_map[atoi(vesc_id)];
                    list<msg_data> set_pos = pos_set_map[atoi(vesc_id)];
                    while (!pos.empty() || !set_pos.empty()) {
                        if (!pos.empty()) { file << pos.front().time << " " << pos.front().data << " "; pos.pop_front();}
                        if (!set_pos.empty()) { file << set_pos.front().time << " " << set_pos.front().data; set_pos.pop_front();}
                        file << endl;
                    }
                } break;
                case 2: //vel
                    //todo write vel data to file
                    break;
                case 3: //tq
                    break;
            }
            file.close();
            system("gnuplot -e \"set terminal png; set output 'image.png'; plot 'data.txt' using 1:2 title 'Response' with lines, 'data.txt' using 3:4 title 'input' with lines\"");
            gtk_image_set_from_file(image, "/home/sean/pid_util/image.png");
            std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        }
        sleep(1);
    }
    g_free(vesc_id);
}

int main (int   argc,
      char *argv[])
{
    //GtkBuilder *builder;
    GObject *window;
    GObject *button;
    GError *error = NULL;

    gtk_init (&argc, &argv);
    
    //load data.txt with default data
    
    ofstream file("data.txt");
    file << 0 << " " << 0 << " " << 0 << " " << 0 << endl;
    file.close();

    /* Construct a GtkBuilder instance and load our UI description */
    builder = gtk_builder_new ();
    if (gtk_builder_add_from_file (builder, "util-window.ui", &error) == 0)
    {
        g_printerr ("Error loading file: %s\n", error->message);
        g_clear_error (&error);
        return 1;
    }
        

    set_combo_box_text_model(GTK_COMBO_BOX_TEXT(gtk_builder_get_object(builder, "ID")));
    //spin up can thread
    std::thread can_recv(can_recv_thd);
    can_recv.detach();
    std::thread plot_data(plot_data_thd);
    plot_data.detach();


     button = gtk_builder_get_object (builder, "PID_load");
     g_signal_connect (button, "clicked", G_CALLBACK (send_pid_act), NULL);

    //start gnuplot to plot data by using s system call
    system("gnuplot -e \"set terminal png; set output 'image.png'; plot 'data.txt' using 1:2 with lines\"");

    // button = gtk_builder_get_object (builder, "quit");
    // g_signal_connect (button, "clicked", G_CALLBACK (gtk_main_quit), NULL);

    gtk_main ();

    return 0;
}
