typedef struct
{
    int8_t                          rssi;
    uint32_t                        time_stamp;
    uint8_t                         data[31];
} ring_struct_t;

uint8_t rb_size();
void rb_insert(uint8_t *data, int8_t rssi, uint32_t time_stamp);
void rb_reset();
void rb_print();
void rb_print_num(uint8_t offset, uint8_t max);
