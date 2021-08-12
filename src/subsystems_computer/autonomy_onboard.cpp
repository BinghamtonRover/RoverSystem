#include "autonomy.hpp"
#include "suspension.hpp"

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace autonomy {

const float HEX_SIZE = 0.7; // In meters.

const float ALPHA = 0.85f;

const uint8_t SPEED = 40;

//###############################################################

enum TurnDirection { COUNTER_CLOCK, CLOCK };

enum InstructionType { MOVE, TURN };

struct Instruction {
    InstructionType type;
    float amount;
    TurnDirection direction;
};

Instruction program[] = {
    { TURN, -45.0, CLOCK },
    { MOVE, 2.4041630560342613 },
    { TURN, 56.30993247402023, COUNTER_CLOCK },
    { MOVE, 2.8844410203711917 },
    { TURN, -4.222230206142683, CLOCK },
    { MOVE, 1.4940548852033517 },
    { TURN, -66.35437736489811, CLOCK },
    { MOVE, 2.0195544062985773 },
    { TURN, 3.814074834290352, COUNTER_CLOCK },
    { MOVE, 3.9086570583769564 },
    { TURN, -101.34941956598566, CLOCK },
    { MOVE, 2.845645796651439 },
    { TURN, -182.32636180357505, CLOCK },
    { MOVE, 6.405279072764901 },
    { TURN, -203.64007322792162, CLOCK },
    { MOVE, 3.067409330363328 },
    { TURN, 83.30461722309516, COUNTER_CLOCK },
    { MOVE, 2.3157936004747914 },
};

int current_instruction = 0;
float start_x = 0;
float start_z = 0;

//###############################################################

const int offsets[6][2] = { { 0, 1 }, { 0, -1 }, { 1, -1 }, { -1, 1 }, { 1, 0 }, { -1, 0 } };

const float angles[6] = { 180.0f, 0.0f, -45.0f, 135.0f, -135.0f, 45.0f };

int abs(int v) {
    return v < 0 ? -v : v;
}

void pos_to_grid_square(float x, float z, int* out_q, int* out_r) {
    // Get the current hexagon.
    float hq = (2.0f / 3.0f) * x / HEX_SIZE;
    float hr = ((-1.0f / 3.0f) * x + (sqrtf(3) / 3.0f) * z) / HEX_SIZE;

    *out_q = (int) hq;
    *out_r = (int) hr;
}

void grid_square_to_pos(int q, int r, float* out_x, float* out_z) {
    *out_x = HEX_SIZE * (sqrtf(3) * (float) q + (sqrt(3) / 2.0f) * (float) r);
    *out_z = HEX_SIZE * (3.0f / 2.0f) * (float) r;
}

const int TQ = 0;
const int TR = -2;

const int GS = 6;

int occupancies[GS * GS];

int get_at(int q, int r) {
    q += GS / 2;
    r += GS / 2;

    return occupancies[r * GS + q];
}

void set_at(int q, int r, int v) {
    q += GS / 2;
    r += GS / 2;

    occupancies[r * GS + q] = v;
}

int distance(int q0, int r0, int q1, int r1) {
    return (abs(q0 - q1) + abs(q0 + r0 - q1 - r1) + abs(r0 - r1)) / 2;
}

float calc_cost(float prob, int dist) {
    return ALPHA * prob + (1.0f - ALPHA) * (float) dist;
}

enum State { THINKING, TRACKING, MOVING, DONE };

State current_state = THINKING;

int move_q, move_r;
int move_offset_idx;

TurnDirection current_turn_direction;

void update(
    unsigned int time, float x, float y, float z, float pitch, float yaw, float roll, std::vector<long> lidar_points) {
    // printf("At time %u: pos (%f, %f, %f), rot (%f, %f, %f)\n", time, x, y, z, pitch, yaw, roll);
    if (current_instruction >= sizeof(program) / sizeof(Instruction)) return;

    Instruction current = program[current_instruction];

    switch (current.type) {
        case MOVE: {
            // printf("MOVE instruction %d. We at (%f, %f) and started at (%f, %f)\n", current_instruction, x, z,
            // start_x, start_z);

            float dist = sqrtf((x - start_x) * (x - start_x) + (z - start_z) * (z - start_z));

            if (fabsf(dist - current.amount) < 0.1f) {
                current_instruction++;

                suspension::update(suspension::LEFT, suspension::FORWARD, 0);
                suspension::update(suspension::RIGHT, suspension::FORWARD, 0);
                suspension::update(suspension::LEFT, suspension::FORWARD, 0);
                suspension::update(suspension::RIGHT, suspension::FORWARD, 0);

                start_x = x;
                start_z = z;

                return;
            }

            suspension::update(suspension::LEFT, suspension::FORWARD, SPEED);
            suspension::update(suspension::RIGHT, suspension::FORWARD, SPEED);
            suspension::update(suspension::LEFT, suspension::FORWARD, SPEED);
            suspension::update(suspension::RIGHT, suspension::FORWARD, SPEED);

            return;
        }
        case TURN: {
            // printf("TURN instruction %d. We at %f and want to get to %f\n", current_instruction, yaw,
            // current.amount);

            if (fabsf(yaw - current.amount) < 5.0f) {
                current_instruction++;

                suspension::update(suspension::LEFT, suspension::FORWARD, 0);
                suspension::update(suspension::RIGHT, suspension::FORWARD, 0);
                suspension::update(suspension::LEFT, suspension::FORWARD, 0);
                suspension::update(suspension::RIGHT, suspension::FORWARD, 0);

                start_x = x;
                start_z = z;

                return;
            }

            if (current.direction == COUNTER_CLOCK) {
                suspension::update(suspension::LEFT, suspension::BACKWARD, SPEED);
                suspension::update(suspension::RIGHT, suspension::FORWARD, SPEED);
                suspension::update(suspension::LEFT, suspension::BACKWARD, SPEED);
                suspension::update(suspension::RIGHT, suspension::FORWARD, SPEED);
            } else {
                suspension::update(suspension::LEFT, suspension::FORWARD, SPEED);
                suspension::update(suspension::RIGHT, suspension::BACKWARD, SPEED);
                suspension::update(suspension::LEFT, suspension::FORWARD, SPEED);
                suspension::update(suspension::RIGHT, suspension::BACKWARD, SPEED);
            }

            return;
        }
    }

    return;

    int q;
    int r;
    pos_to_grid_square(x, z, &q, &r);
    // printf("current hex: (%d, %d)\n", q, r);

#if 0
	memset(occupancies, 0, sizeof(occupancies));

	for (int i = 0; i < (int)lidar_points.size(); i++) {
		long point = lidar_points[i];

		// float angle = 225.0f - (float)i;
		float angle = 225.0f - (float)i;
		float theta = angle * M_PI / 180.0f;

		theta += M_PI;

		if (point < 100 || point > 10000) continue;

		int pq, pr;

		float px = x + ((float) point / 1000.0f) * cosf(theta);
		float pz = z + ((float) point / 1000.0f) * sinf(theta);

		pos_to_grid_square(px, pz, &pq, &pr);

		set_at(pq, pr, get_at(pq, pr) + 1);
	}

	int max_q = q + offsets[0][0], max_r = r + offsets[0][1];

	for (int i = 0; i < 6; i++) {
		int aq = q + offsets[i][0];
		int ar = r + offsets[i][1];

		if (aq == TQ && ar == TR) {
			printf("YOU CAN JUST MOVE TO THE TARGET!\n");
			return;
		}

		if (get_at(aq, ar) > get_at(max_q, max_r)) {
			max_q = aq;
			max_r = ar;
		}

		printf("Adjacent cell (%d, %d) has occ %d and distance %d\n", aq, ar, get_at(aq, ar), distance(TQ, TR, aq, ar));
	}

	return;
#endif

#if 1

    if (current_state == MOVING) {
        // Recalculate x and z so that we are slightly further back.
        float back_extent_yaw = yaw + 180.0f;

        float dist = 0.2f;

        float dx = x + dist * cosf(back_extent_yaw * M_PI / 180.0f);
        float dz = z - dist * sinf(back_extent_yaw * M_PI / 180.0f);

        int uuhq, uuhr;

        pos_to_grid_square(dx, dz, &uuhq, &uuhr);

        if (uuhq == move_q && uuhr == move_r) {
            suspension::update(suspension::LEFT, suspension::FORWARD, 0);
            suspension::update(suspension::RIGHT, suspension::FORWARD, 0);

            if (q == TQ && r == TR) {
                printf("done.\n");
                current_state = DONE;
                return;
            }

            printf("MOVED! GOING BACK TO THINKING!\n");
            current_state = THINKING;

            return;
        }

        printf("Moving, not yet in the correct thingy.\n");

        suspension::update(suspension::LEFT, suspension::FORWARD, SPEED);
        suspension::update(suspension::RIGHT, suspension::FORWARD, SPEED);

        return;
    } else if (current_state == TRACKING) {
        float tx, tz;
        grid_square_to_pos(q, r, &tx, &tz);

        printf("Tracking... Current angle %f, target angle %f\n", yaw, angles[move_offset_idx]);

        if (fabsf(yaw - angles[move_offset_idx]) < 5.0f) {
            printf("DONE ROTATING! MOVING\n");

            suspension::update(suspension::LEFT, suspension::FORWARD, 0);
            suspension::update(suspension::RIGHT, suspension::FORWARD, 0);

            current_state = MOVING;
            return;
        }

        if (current_turn_direction == CLOCK) {
            suspension::update(suspension::LEFT, suspension::FORWARD, SPEED);
            suspension::update(suspension::RIGHT, suspension::BACKWARD, SPEED);
        } else {
            suspension::update(suspension::LEFT, suspension::BACKWARD, SPEED);
            suspension::update(suspension::RIGHT, suspension::FORWARD, SPEED);
        }

        return;
    } else if (current_state == DONE) {
        suspension::update(suspension::LEFT, suspension::FORWARD, 0);
        suspension::update(suspension::RIGHT, suspension::FORWARD, 0);

        return;
    }

    // Thinking.

    memset(occupancies, 0, sizeof(occupancies));

    for (int i = 0; i < (int) lidar_points.size(); i++) {
        long point = lidar_points[i];

        float angle = 225.0f - (float) i;
        float theta = angle * M_PI / 180.0f;

        theta += M_PI + (yaw * M_PI / 180.0f);

        if (point < 100 || point > 10000) continue;

        int pq, pr;

        float px = x + ((float) point / 1000.0f) * cosf(theta);
        float pz = z + ((float) point / 1000.0f) * sinf(theta);

        pos_to_grid_square(px, pz, &pq, &pr);

        set_at(pq, pr, get_at(pq, pr) + 1);
    }

    int max_q = q + offsets[0][0], max_r = r + offsets[0][1];

    for (int i = 0; i < 6; i++) {
        int aq = q + offsets[i][0];
        int ar = r + offsets[i][1];

        if (get_at(aq, ar) > get_at(max_q, max_r)) {
            max_q = aq;
            max_r = ar;
        }

        printf("Adjacent cell (%d, %d) has occ %d and distance %d\n", aq, ar, get_at(aq, ar), distance(TQ, TR, aq, ar));
    }

    printf("Max is at (%d, %d) of %d\n", max_q, max_r, get_at(max_q, max_r));

    int bad_distance = distance(TQ, TR, q, r) + 1;

    float min_cost = 1;
    int best_offset_idx = 0;

    for (int i = 0; i < 6; i++) {
        int aq = q + offsets[i][0];
        int ar = r + offsets[i][1];

        if (distance(TQ, TR, aq, ar) == bad_distance) continue;

        int occ = get_at(aq, ar);
        float prob = (float) occ / (float) get_at(max_q, max_r);

        float cost = calc_cost(prob, distance(TQ, TR, aq, ar));

        printf(" == Cell (%d, %d) has occ prob %f and distance %d\n", aq, ar, prob, distance(TQ, TR, aq, ar));
        printf("                  has cost %f\n", cost);

        if (cost < min_cost) {
            min_cost = cost;

            best_offset_idx = i;
        }
    }

    printf("MOVE TO (%d, %d) (%f)\n", offsets[best_offset_idx][0] + q, offsets[best_offset_idx][1] + r, min_cost);

    move_q = q + offsets[best_offset_idx][0];
    move_r = r + offsets[best_offset_idx][1];
    move_offset_idx = best_offset_idx;

    current_state = TRACKING;

    if (angles[move_offset_idx] < yaw) {
        current_turn_direction = CLOCK;
    } else {
        current_turn_direction = COUNTER_CLOCK;
    }

    suspension::update(suspension::LEFT, suspension::FORWARD, 0);
    suspension::update(suspension::RIGHT, suspension::FORWARD, 0);

#endif
}

} // namespace autonomy
