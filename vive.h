#pragma once

#include <sys/shm.h>

#define SH_MEM_ADDR 22222

struct Position {
    float x, y, z, yaw;
    int version;
};

Position *position;

void init_shared_memory() {
    int act_segment_id;
    struct shmid_ds act_shmbuffer;
    int act_segment_size;
    const int act_shared_segment_size = 0x6400;
    int act_smhkey = SH_MEM_ADDR;

    act_segment_id = shmget(act_smhkey, act_shared_segment_size, IPC_CREAT | 0666);
    position = (Position *) shmat(act_segment_id, 0, 0);
    shmctl(act_segment_id, IPC_STAT, &act_shmbuffer);
    act_segment_size = act_shmbuffer.shm_segsz;
}
