#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>

FILE *csv_fp;
static char filename[256];

FILE* open_csv()
{
    csv_fp = fopen(filename, "a");
    if (!csv_fp) {
        perror("fopen");
        return NULL;
    }

    return csv_fp;
}

void close_csv()
{
    fclose(csv_fp);
}


int create_csv()
{
    FILE *fp;
    char line[512];
    char device[128], mountpoint[128], fstype[32];
    char target_mount[128] = "";

    fp = fopen("/proc/mounts", "r");
    if (!fp) {
        perror("fopen");
        return 1;
    }

    while (fgets(line, sizeof(line), fp)) {
        sscanf(line, "%s %s %s", device, mountpoint, fstype);

        if (strncmp(device, "/dev/sd", 7) == 0 &&
            strstr(mountpoint, "/media/") != NULL) {

            strcpy(target_mount, mountpoint);
            break;
        }
    }
    fclose(fp);

    if (strlen(target_mount) == 0) {
        printf("can't find USB mount point\n");
        return 1;
    }

    printf("mount: %s\n", target_mount);

    DIR *dir = opendir(target_mount);
    if (!dir) {
        perror("opendir");
        return 1;
    }

    struct dirent *entry;
    int max = 0;

    while ((entry = readdir(dir)) != NULL) {
        int num;

        if (sscanf(entry->d_name, "thermal_plot%d.csv", &num) == 1) {
            if (num > max) max = num;
        }
    }

    closedir(dir);

    snprintf(filename, sizeof(filename),
             "%s/thermal_plot%d.csv", target_mount, max + 1);

    FILE *out = fopen(filename, "w");
    if (!out) {
        perror("fopen");
        return 1;
    }

    printf("create: %s\n", filename);

    fclose(out);

    return 0;
}


