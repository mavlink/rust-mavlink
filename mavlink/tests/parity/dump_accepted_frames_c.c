#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <all/mavlink.h>

static void append_byte(char *out, size_t *offset, uint8_t byte)
{
    static const char hex[] = "0123456789abcdef";

    out[(*offset)++] = hex[byte >> 4];
    out[(*offset)++] = hex[byte & 0x0f];
}

static void append_payload(char *out, size_t *offset, const mavlink_message_t *message)
{
    const uint8_t *payload = (const uint8_t *)_MAV_PAYLOAD(message);

    for (uint8_t i = 0; i < message->len; i++) {
        append_byte(out, offset, payload[i]);
    }
}

static void print_frame(const mavlink_message_t *message)
{
    char out[(MAVLINK_MAX_PACKET_LEN * 2) + 1];
    size_t offset = 0;

    append_byte(out, &offset, message->magic);
    append_byte(out, &offset, message->len);

    if (message->magic == MAVLINK_STX_MAVLINK1) {
        append_byte(out, &offset, message->seq);
        append_byte(out, &offset, message->sysid);
        append_byte(out, &offset, message->compid);
        append_byte(out, &offset, (uint8_t)(message->msgid & 0xff));
    } else {
        append_byte(out, &offset, message->incompat_flags);
        append_byte(out, &offset, message->compat_flags);
        append_byte(out, &offset, message->seq);
        append_byte(out, &offset, message->sysid);
        append_byte(out, &offset, message->compid);
        append_byte(out, &offset, (uint8_t)(message->msgid & 0xff));
        append_byte(out, &offset, (uint8_t)((message->msgid >> 8) & 0xff));
        append_byte(out, &offset, (uint8_t)((message->msgid >> 16) & 0xff));
    }

    append_payload(out, &offset, message);
    append_byte(out, &offset, (uint8_t)(message->checksum & 0xff));
    append_byte(out, &offset, (uint8_t)(message->checksum >> 8));

    if (message->magic != MAVLINK_STX_MAVLINK1
        && (message->incompat_flags & MAVLINK_IFLAG_SIGNED) != 0) {
        for (uint8_t i = 0; i < MAVLINK_SIGNATURE_BLOCK_LEN; i++) {
            append_byte(out, &offset, message->signature[i]);
        }
    }

    out[offset] = '\0';
    puts(out);
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: dump_accepted_frames_c <mavlink-stream>\n");
        return 2;
    }

    FILE *file = fopen(argv[1], "rb");
    if (file == NULL) {
        fprintf(stderr, "failed to open %s: %s\n", argv[1], strerror(errno));
        return 2;
    }

    mavlink_message_t message;
    mavlink_status_t status;
    memset(&message, 0, sizeof(message));
    memset(&status, 0, sizeof(status));

    int byte;
    while ((byte = fgetc(file)) != EOF) {
        if (mavlink_parse_char(
                MAVLINK_COMM_0,
                (uint8_t)byte,
                &message,
                &status)) {
            print_frame(&message);
        }
    }

    if (ferror(file)) {
        fprintf(stderr, "failed to read %s: %s\n", argv[1], strerror(errno));
        fclose(file);
        return 2;
    }

    fclose(file);
    return 0;
}
