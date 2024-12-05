#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979323846
#define MAX_POINTS 1001 // t_end / dt + 1

int main() {
    double t_end = 10.0;
    double dt = 0.01;
    int n_points = (int)(t_end / dt) + 1;

    // Declare static arrays
    double t_road[MAX_POINTS];
    double delta[MAX_POINTS];
    double beta[MAX_POINTS];
    double yaw[MAX_POINTS];

    // Initialize time array
    for (int i = 0; i < n_points; i++) {
        t_road[i] = i * dt;
    }

    // Vehicle parameters
    double degree = 2.0;
    double delta_dot = degree * PI / 180.0;
    double steering_time = 1.0;
    double V = 15.0; // Constant speed
    double m = 2000.0;
    double Lf = 1.4, Lr = 1.6, L = Lf + Lr;
    double Cf = 13525.714, Cr = 15166.667;

    // Steering profile
    for (int i = 0; i < n_points; i++) {
        if (t_road[i] <= steering_time) {
            delta[i] = 0.0;
        } else if (t_road[i] <= steering_time + 2.5) {
            delta[i] = delta_dot * sin(PI / 2 * (t_road[i] - steering_time) / (0.25 * 2.5));
        } else if (t_road[i] <= steering_time + 3.3) {
            delta[i] = 0.0;
        } else if (t_road[i] <= steering_time + 5.8) {
            delta[i] = -delta_dot * sin(PI / 2 * (t_road[i] - (steering_time + 1.7)) / (0.25 * 2.5));
        } else {
            delta[i] = 0.0;
        }
    }

    // Calculate desired slip angle and yaw rate
    FILE *file = fopen("results.txt", "w");
    if (file == NULL) {
        perror("Error opening file");
        return 1;
    }

    fprintf(file, "Time(s)\tSideSlip(rad)\tYawRate(rad/s)\n");
    for (int i = 0; i < n_points; i++) {
        beta[i] = (Lr - (Lf * m * V * V) / (2 * Cr * (Lf + Lr))) /
                  ((Lf + Lr) + (m * V * V * (Lr * Cr - Lf * Cf)) / (2 * Cf * Cr * (Lf + Lr))) * delta[i];
        yaw[i] = V / (Lf + Lr + (m * V * V * (Lr * Cr - Lf * Cf)) / (2 * Cf * Cr * L)) * delta[i];
        fprintf(file, "%6.2f\t%12.6f\t%12.6f\n", t_road[i], beta[i], yaw[i]);
    }

    fclose(file);

    return 0;
}
