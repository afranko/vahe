
typedef struct mac_general_message_format{
    uint8_t frame_ctrl[2];
    uint8_t seq_id;
    uint16_t dpan_id;   //TODO kell-e vagy opcionális
    uint16_t dest_addr;
    uint16_t span_id;   //TODO kell-e vagy opcionális
    uint16_t src_addr;
    uint8_t auxs_header[];  //TODO kell-e vagy opcionális
    uint8_t payload[];  //TODO megoldani
    uint8_t mfooter[2];
};

// TODO AZ üzenetek gecire DMA kompatibilis memóriába menjenek :D
