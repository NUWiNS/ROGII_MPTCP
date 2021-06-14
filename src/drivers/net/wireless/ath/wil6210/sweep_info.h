#ifndef SWEEP_INFO_H_
#define SWEEP_INFO_H_
/* ATTENTION: This is a _shared
 * Both driver and firmware do 
 * but we still keep everything
 */
// ----------------------------
// Shared between driver and fi
// ----------------------------
#define SWEEP_DUMP_SIZE 256
/*typedef struct {
    uint16_t sector_id;
    int16_t snr;
    uint32_t rssi;
    uint8_t macaddr[6];
    uint8_t flags[2];
} sector_info_t;*/

typedef struct {
        uint8_t src[6];
        uint8_t swp[3];
        int16_t snr;
        uint16_t ctr;
        uint16_t flags;
 } sector_info_t;

typedef struct {
    uint32_t cur_pos;
    uint32_t ctr_pkts;
    uint32_t ctr_swps;
    sector_info_t dump[SWEEP_DUMP_SIZE];
} sweep_dump_t;

#define PTR_MEM_SWEEP_CUR_FEEDBACK 0x93D000
#define PTR_MEM_SWEEP_DUMP 0x93D018
// ---------------------------------------------------------------------------
// Used only by firmware
// ---------------------------------------------------------------------------
#define PTR_SEL_SWEEP_SECTOR_INFO 0x800F1C
// #define PTR_CUR_SWEEP_SECTOR
#define PTR_CUR_SWEEP_SECTOR_INFO 0x801038
#define PTR_CUR_FRAME_BUFFER 0x804010

typedef struct{
        uint8_t macaddr [6];
        uint16_t sector_id;
        uint16_t snr;
        uint8_t reserved[2];
} custom_sweep_feedback_t;

typedef struct {
   // bool force_sector;
   // bool force_rssi;
   // uint16_t sector_id;
   // int32_t snr;
   // uint32_t crrsi;
    custom_sweep_feedback_t sweep_feedback [8];
} sweep_overwrite_t;

#define PTR_MEM_SWEEP_OVERWRITE 0x93D00C
// ----------------------------
// Used only by driver
// ----------------------------
#define PTR_SWEEP_STATS 0x9406FC
typedef struct {
    uint32_t ctr_valid;
    uint32_t ctr_overflow;
    uint16_t ctr_null;
    uint16_t ctr_missed;
    uint8_t reserved;
    uint8_t flag;
    uint16_t reserved2;
} sweep_stats_t;
#endif /* SWEEP_INFO_H_ */