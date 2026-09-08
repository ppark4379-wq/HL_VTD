// Compile with -include /path/to/VtdToolkit/viRDBIcd.h; SDK is not redistributed.
#include <cstddef>
static_assert(sizeof(RDB_MSG_HDR_t)==24, "RDB message header changed");
static_assert(sizeof(RDB_MSG_ENTRY_HDR_t)==16, "RDB entry header changed");
static_assert(sizeof(RDB_OBJECT_STATE_BASE_t)==112, "RDB object base changed");
static_assert(sizeof(RDB_OBJECT_STATE_t)==208, "RDB object extension changed");
static_assert(offsetof(RDB_OBJECT_STATE_BASE_t,geo)==40, "geometry offset changed");
static_assert(offsetof(RDB_OBJECT_STATE_BASE_t,pos)==64, "position offset changed");
static_assert(sizeof(RDB_ROAD_POS_t)==40, "road position changed");
static_assert(sizeof(RDB_LANE_INFO_t)==56, "lane info changed");
static_assert(offsetof(RDB_LANE_INFO_t,playerId)==48, "lane player offset changed");
static_assert(offsetof(RDB_VEHICLE_SYSTEMS_t,lightMask)==4, "light mask offset changed");
int main() { return 0; }
