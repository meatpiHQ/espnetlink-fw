/**
 * @file restart_tracker_cli.c
 * @brief The component's CLI command (`restart_tracker`) — registered
 *        into cmdline_manager by restart_tracker_register_cli() (main
 *        wires it in CLI compositions only). Legacy option interface
 *        preserved (-l/--latest, -a/--history, -p/--pending,
 *        -n/--count, --panic); bare = counters + latest. v6 flags are
 *        caller-defined, so they print as hex (legacy had named flags).
 */
#include <inttypes.h>
#include <stdio.h>
#include <time.h>

#include "argtable3/argtable3.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cmdline_manager.h"

#include "restart_tracker.h"

static struct
{
    struct arg_lit *latest;
    struct arg_lit *history;
    struct arg_lit *pending;
    struct arg_lit *panic;
    struct arg_int *count;
    struct arg_end *end;
} s_args;

static void format_timestamp(int64_t ts, bool valid, char *buf,
                             size_t len)
{
    struct tm utc;
    time_t t = (time_t)ts;

    if (!valid || ts <= 0)
    {
        snprintf(buf, len, "unsynced");
        return;
    }

    gmtime_r(&t, &utc);

    if (strftime(buf, len, "%Y-%m-%dT%H:%M:%SZ", &utc) == 0)
    {
        snprintf(buf, len, "%" PRId64, ts);
    }
}

static void print_record(const restart_tracker_record_t *r)
{
    char boot_ts[32];
    char req_ts[32];

    format_timestamp(r->boot_timestamp, r->time_valid != 0, boot_ts,
                     sizeof(boot_ts));
    format_timestamp(r->request_timestamp, r->request_timestamp > 0,
                     req_ts, sizeof(req_ts));
    cmdline_printf("[%" PRIu32 "] reset=%s planned=%s source=%s "
                   "flags=0x%" PRIx32 " request=%s uptime_ms=%" PRIu64
                   " boot=%s\n",
                   r->sequence,
                   restart_tracker_reset_reason_to_str(
                       r->actual_reset_reason),
                   restart_tracker_planned_reason_to_str(
                       (restart_tracker_planned_reason_t)
                           r->planned_reason),
                   restart_tracker_source_to_str(
                       (restart_tracker_source_t)r->source),
                   r->flags, req_ts, r->request_uptime_ms, boot_ts);
}

static int print_history(int max_count)
{
    static restart_tracker_state_t st EXT_RAM_BSS_ATTR; /* ~600 B */

    if (restart_tracker_get_state(&st) != ESP_OK)
    {
        cmdline_printf("Error: restart tracker unavailable\n");
        return 1;
    }

    cmdline_printf("Boots: %" PRIu32 ", unexpected resets: %" PRIu32
                   "\n", st.boot_count, st.unexpected_reset_count);

    int printed = 0;

    for (uint32_t n = 0; n < RESTART_TRACKER_HISTORY_LEN &&
                         printed < max_count; n++)
    {
        uint32_t idx = (st.latest_history_index +
                        RESTART_TRACKER_HISTORY_LEN - n) %
                       RESTART_TRACKER_HISTORY_LEN;

        if (st.history[idx].sequence == 0)
        {
            continue;
        }

        print_record(&st.history[idx]);
        printed++;
    }

    return 0;
}

static int print_pending(void)
{
    static restart_tracker_state_t st EXT_RAM_BSS_ATTR;

    if (restart_tracker_get_state(&st) != ESP_OK)
    {
        cmdline_printf("Error: restart tracker unavailable\n");
        return 1;
    }

    const restart_tracker_pending_t *p = &st.pending_restart;

    if (!p->valid)
    {
        cmdline_printf("No pending restart intent\n");
        return 0;
    }

    char req_ts[32];

    format_timestamp(p->requested_timestamp, p->time_valid != 0, req_ts,
                     sizeof(req_ts));
    cmdline_printf("Pending: planned=%s source=%s flags=0x%" PRIx32
                   " request=%s uptime_ms=%" PRIu64 "\n",
                   restart_tracker_planned_reason_to_str(
                       (restart_tracker_planned_reason_t)
                           p->planned_reason),
                   restart_tracker_source_to_str(
                       (restart_tracker_source_t)p->source),
                   p->flags, req_ts, p->requested_uptime_ms);
    return 0;
}

static int cmd_restart_tracker(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&s_args);

    if (nerrors != 0)
    {
        arg_print_errors(stderr, s_args.end, argv[0]);
        return 1;
    }

    if (s_args.panic->count > 0)
    {
        /* deliberately die UNANNOUNCED: the next boot must record an
           unplanned panic — this is the tracker's self-test */
        cmdline_printf("Triggering test panic...\n");
        vTaskDelay(pdMS_TO_TICKS(300));
        abort();
    }

    if (s_args.latest->count > 0)
    {
        restart_tracker_record_t rec;

        if (restart_tracker_get_latest_record(&rec) != ESP_OK)
        {
            cmdline_printf("Error: no boot record\n");
            return 1;
        }

        print_record(&rec);
        return 0;
    }

    if (s_args.pending->count > 0)
    {
        return print_pending();
    }

    int count = RESTART_TRACKER_HISTORY_LEN;

    if (s_args.count->count > 0 && s_args.count->ival[0] > 0)
    {
        count = s_args.count->ival[0];
    }

    if (s_args.history->count > 0)
    {
        return print_history(count);
    }

    return print_history(1); /* bare: counters + the latest record */
}

esp_err_t restart_tracker_register_cli(void)
{
    static const esp_console_cmd_t CMD =
    {
        .command = "restart_tracker",
        .help = "Inspect retained restart tracker state",
        .hint = "Options: -l/--latest, -a/--history, -p/--pending, "
                "-n/--count <n>, --panic",
        .func = cmd_restart_tracker,
        .argtable = &s_args,
    };

    s_args.latest = arg_lit0("l", "latest", "Show the latest boot record");
    s_args.history = arg_lit0("a", "history", "Show the boot history");
    s_args.pending = arg_lit0("p", "pending", "Show pending restart intent");
    s_args.panic = arg_lit0(NULL, "panic",
                            "Trigger a test panic (unplanned reset)");
    s_args.count = arg_int0("n", "count", "<n>", "Max history records");
    s_args.end = arg_end(5);
    return cmdline_manager_register(&CMD);
}
