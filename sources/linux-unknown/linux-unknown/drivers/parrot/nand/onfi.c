/**
 *
 *       @file  onfi.c
 *
 *      @brief  
 *
 *     @author  Gregor Boirie <gregor.boirie@parrot.com>
 *       @date  22-Apr-2009
 *
 *        $Id: onfi.c,v 1.8 2009-06-02 14:02:21 gboirie Exp $
 *
 *       Copyright (C) 2008 Parrot S.A.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/crc16.h>
#include <linux/mtd/mtd.h>
#include "onfi.h"

static                      LIST_HEAD(onfi_settings);
static struct onfi_setting* onfi_default;

static
struct onfi_timings const
onfi_unknown_timings = {
        .tALH = -1,
        .tALS = -1,
        .tCEA = -1,
        .tCEH = -1,
        .tCH = -1,
        .tCLH = -1,
        .tCLR = -1,
        .tCLS = -1,
        .tCS = -1,
        .tDH = -1,
        .tDS = -1,
        .tRC = -1,
        .tREA = -1,
        .tREH = -1,
        .tRHZ = -1,
        .tRP = -1,
        .tRR = -1,
        .tWC = -1,
        .tWH = -1,
        .tWHR = -1,
        .tWP = -1,
        .toRST = -1,
        .twRST = -1,
        .teRST = -1
};

static
struct onfi_timings const
onfi_timings_modes[] = {
    [0] = {
        .tALH = 20,
        .tALS = 50,
        .tCEA = 100,
        .tCEH = 0,
        .tCH = 20,
        .tCLH = 20,
        .tCLR = 20,
        .tCLS = 50,
        .tCS = 70,
        .tDH = 20,
        .tDS = 40,
        .tRC = 100,
        .tREA = 40,
        .tREH = 30,
        .tRHZ = 200,
        .tRP = 50,
        .tRR = 39,
        .tWC = 100,
        .tWH = 30,
        .tWHR = 120,
        .tWP = 50,
        .toRST = 1000,
        .twRST = 1000,
        .teRST = 1000
    },
    [1] = {
        .tALH = 10,
        .tALS = 25,
        .tCEA = 45,
        .tCEH = 0,
        .tCH = 10,
        .tCLH = 10,
        .tCLR = 10,
        .tCLS = 25,
        .tCS = 35,
        .tDH = 10,
        .tDS = 20,
        .tRC = 50,
        .tREA = 30,
        .tREH = 15,
        .tRHZ = 100,
        .tRP = 25,
        .tRR = 20,
        .tWC = 45,
        .tWH = 15,
        .tWHR = 80,
        .tWP = 25,
        .toRST = 5,
        .twRST = 10,
        .teRST = 500
    },
    [2] = {
        .tALH = 10,
        .tALS = 15,
        .tCEA = 30,
        .tCEH = 0,
        .tCH = 10,
        .tCLH = 10,
        .tCLR = 10,
        .tCLS = 15,
        .tCS = 25,
        .tDH = 5,
        .tDS = 15,
        .tRC = 35,
        .tREA = 25,
        .tREH = 15,
        .tRHZ = 100,
        .tRP = 17,
        .tRR = 20,
        .tWC = 35,
        .tWH = 15,
        .tWHR = 80,
        .tWP = 17,
        .toRST = 5,
        .twRST = 10,
        .teRST = 500
    },
    [3] = {
        .tALH = 5,
        .tALS = 10,
        .tCEA = 25,
        .tCEH = 0,
        .tCH = 5,
        .tCLH = 5,
        .tCLR = 10,
        .tCLS = 10,
        .tCS = 25,
        .tDH = 5,
        .tDS = 10,
        .tRC = 30,
        .tREA = 20,
        .tREH = 10,
        .tRHZ = 100,
        .tRP = 15,
        .tRR = 20,
        .tWC = 30,
        .tWH = 10,
        .tWHR = 60,
        .tWP = 15,
        .toRST = 5,
        .twRST = 10,
        .teRST = 500
    },
    [4] = {
        .tALH = 5,
        .tALS = 10,
        .tCEA = 25,
        .tCEH = 0,
        .tCH = 5,
        .tCLH = 5,
        .tCLR = 10,
        .tCLS = 10,
        .tCS = 20,
        .tDH = 5,
        .tDS = 10,
        .tRC = 25,
        .tREA = 20,
        .tREH = 10,
        .tRHZ = 100,
        .tRP = 12,
        .tRR = 20,
        .tWC = 25,
        .tWH = 10,
        .tWHR = 60,
        .tWP = 12,
        .toRST = 5,
        .twRST = 10,
        .teRST = 500
    },
    [5] = {
        .tALH = 5,
        .tALS = 10,
        .tCEA = 25,
        .tCEH = 0,
        .tCH = 5,
        .tCLH = 5,
        .tCLR = 10,
        .tCLS = 10,
        .tCS = 15,
        .tDH = 5,
        .tDS = 7,
        .tRC = 20,
        .tREA = 16,
        .tREH = 7,
        .tRHZ = 100,
        .tRP = 10,
        .tRR = 20,
        .tWC = 20,
        .tWH = 7,
        .tWHR = 60,
        .tWP = 10,
        .toRST = 5,
        .twRST = 10,
        .teRST = 500
    }
};

struct onfi_parser_ctx {
    char*                   start;
    char*                   next;
};

typedef void (onfi_parm_handle_t)(struct onfi_setting*, char const*, char const*);

struct onfi_parm_handler {
    char const*         name;
    onfi_parm_handle_t* handle;
};

#define UCHAR_MAX    ((u8)(~0U))
#define CHAR_MAX    ((s8)(UCHAR_MAX >> 1))

#define type_max(var)                                                               \
    ({                                                                              \
        __builtin_choose_expr(__builtin_types_compatible_p(typeof(var),             \
                                                           signed char), CHAR_MAX,  \
        __builtin_choose_expr(__builtin_types_compatible_p(typeof(var),             \
                                                           signed short), SHORT_MAX,\
        ((void) 0)));                                                               \
     })

#define onfi_strtol(parm, name, value)                                  \
    ({                                                                  \
        long _v = simple_strtol(value, NULL, 0);                        \
        typeof(parm) _p = parm;                                         \
        typeof(*_p) _max = type_max(*_p);                               \
        if (unlikely(_v > _max || _v < 0)) {                            \
            pr_info("ONFI: value %ld is invalid for parameter \"%s\"\n",\
                    _v, name);                                          \
            _v = -1;                                                    \
        }                                                               \
        *_p = (typeof(*_p)) _v;                                         \
     })

#define ONFI_DEFINE_TIMING_HANDLER(name)                        \
    void                                                        \
    onfi_handle_ ## name ## _parm(struct onfi_setting* setting, \
                                  char const* name,             \
                                  char const* value)            \
    { onfi_strtol(&setting->timings.name , name, value); }

#define ONFI_PARM_HANDLER_INIT(name) \
    { # name , onfi_handle_ ## name ## _parm }

#define __unused __attribute__((unused))

static
void
onfi_handle_null_parm(struct onfi_setting* setting __unused,
                      char const* name,
                      char const* value __unused)
{
    pr_warning("ONFI: unknown parameter \"%s\"\n", name);
}

static ONFI_DEFINE_TIMING_HANDLER(tALH)
static ONFI_DEFINE_TIMING_HANDLER(tALS)
static ONFI_DEFINE_TIMING_HANDLER(tCEA)
static ONFI_DEFINE_TIMING_HANDLER(tCEH)
static ONFI_DEFINE_TIMING_HANDLER(tCH)
static ONFI_DEFINE_TIMING_HANDLER(tCLH)
static ONFI_DEFINE_TIMING_HANDLER(tCLR)
static ONFI_DEFINE_TIMING_HANDLER(tCLS)
static ONFI_DEFINE_TIMING_HANDLER(tCS)
static ONFI_DEFINE_TIMING_HANDLER(tDH)
static ONFI_DEFINE_TIMING_HANDLER(tDS)
static ONFI_DEFINE_TIMING_HANDLER(tRC)
static ONFI_DEFINE_TIMING_HANDLER(tREA)
static ONFI_DEFINE_TIMING_HANDLER(tREH)
static ONFI_DEFINE_TIMING_HANDLER(tRHZ)
static ONFI_DEFINE_TIMING_HANDLER(tRP)
static ONFI_DEFINE_TIMING_HANDLER(tRR)
static ONFI_DEFINE_TIMING_HANDLER(tWC)
static ONFI_DEFINE_TIMING_HANDLER(tWH)
static ONFI_DEFINE_TIMING_HANDLER(tWHR)
static ONFI_DEFINE_TIMING_HANDLER(tWP)
static ONFI_DEFINE_TIMING_HANDLER(toRST)
static ONFI_DEFINE_TIMING_HANDLER(twRST)
static ONFI_DEFINE_TIMING_HANDLER(teRST)

static
struct onfi_parm_handler const
onfi_parm_handlers[] = {
    { "", &onfi_handle_null_parm },
	ONFI_PARM_HANDLER_INIT(tALH),
	ONFI_PARM_HANDLER_INIT(tALS),
	ONFI_PARM_HANDLER_INIT(tCEA),
	ONFI_PARM_HANDLER_INIT(tCEH),
	ONFI_PARM_HANDLER_INIT(tCH),
	ONFI_PARM_HANDLER_INIT(tCLH),
	ONFI_PARM_HANDLER_INIT(tCLR),
	ONFI_PARM_HANDLER_INIT(tCLS),
	ONFI_PARM_HANDLER_INIT(tCS),
	ONFI_PARM_HANDLER_INIT(tDH),
	ONFI_PARM_HANDLER_INIT(tDS),
	ONFI_PARM_HANDLER_INIT(tRC),
	ONFI_PARM_HANDLER_INIT(tREA),
	ONFI_PARM_HANDLER_INIT(tREH),
	ONFI_PARM_HANDLER_INIT(tRHZ),
	ONFI_PARM_HANDLER_INIT(tRP),
	ONFI_PARM_HANDLER_INIT(tRR),
	ONFI_PARM_HANDLER_INIT(tWC),
	ONFI_PARM_HANDLER_INIT(tWH),
	ONFI_PARM_HANDLER_INIT(tWHR),
	ONFI_PARM_HANDLER_INIT(tWP),
	ONFI_PARM_HANDLER_INIT(toRST),
	ONFI_PARM_HANDLER_INIT(twRST),
	ONFI_PARM_HANDLER_INIT(teRST)
};

static
struct onfi_setting*
onfi_alloc_setting(void)
{
    struct onfi_setting* const setting = kmalloc(sizeof(struct onfi_setting),
                                                 GFP_KERNEL);
    if (unlikely(!  setting))
        return 0;

    INIT_LIST_HEAD(&setting->node);
    setting->name = 0;
    setting->edo = 0;
    setting->mode = 0;
    memcpy(&setting->timings, &onfi_unknown_timings, sizeof(onfi_unknown_timings));

    /*
     * add it to the list of registered settings if related to a specific
     * mtd device
     */
    list_add_tail(&setting->node, &onfi_settings);

    return setting;
}

#define array_count(array) (sizeof(array) / sizeof(array[0]))

static
int
onfi_parse_parms(struct onfi_parser_ctx* context)
{
    char const* const       name = context->start;
    char const*             value;
    char*                   tmp = context->start;
    int                     parm = array_count(onfi_parm_handlers);

    BUG_ON(! tmp);

    while (isalnum(*tmp))
        tmp++;

    if (unlikely(! *tmp)) {
        pr_warning("ONFI: null parameter\n");
        return -EINVAL;
    }

    if (*tmp != '=') {
        pr_warning("ONFI: invalid parameter specification\n");
        return -EINVAL;
    }
    else {
        *tmp = '\0';
        while (parm) {
            parm--;
            if (! strcmp(name, onfi_parm_handlers[parm].name))
                break;
        }
    }

    value = ++tmp;
    while (isalnum(*tmp))
       tmp++;

    if (*tmp == ',') {
        struct onfi_parser_ctx ctx = {
            .start = tmp + 1
        };
        int ret = onfi_parse_parms(&ctx);
        if (ret)
            return ret;

        context->next = ctx.next;
    }
    else {
        struct onfi_setting*    setting;

        if (unlikely(! name && onfi_default))
            /* there may be a single default settings entry */
            return -EEXIST;

        if (! list_empty(&onfi_settings)) {
            list_for_each_entry(setting, &onfi_settings, node) {
                if (! strcmp(name, setting->name))
                    /* there may be a single settings entry per mtd device */
                    return -EEXIST;
            }
        }

        /* it is now safe to alloc a new settings entry */
        setting = onfi_alloc_setting();
        if (! setting)
            return -ENOMEM;

        if (! name)
            /* set the default settings entry */
            onfi_default = setting;

        if (*tmp)
            context->next = tmp + 1; 
        else
            context->next = tmp; 
    }

    BUG_ON(list_empty(&onfi_settings));

    *tmp = '\0';
    onfi_parm_handlers[parm].handle(list_entry(onfi_settings.prev,
                                               struct onfi_setting, node),
                                    name,
                                    value);
    return 0;
}

static
int
onfi_parse_mtdid(struct onfi_parser_ctx* context)
{
    char* tmp = context->start;

    BUG_ON(! tmp);

    while (isalnum(*tmp))
        tmp++;

    if (unlikely(! *tmp)) {
        pr_warning("ONFI: invalid mtd specification\n");
        return -EINVAL;
    }

    if (*tmp != ':') {
        /* default parameters set for all NAND mtd devices */
        context->next = context->start;
        context->start = 0;
    }
    else {
        *tmp = '\0';
        context->next = tmp + 1;
    }

    return 0;
}

static
int
onfi_parse_mtd(struct onfi_parser_ctx* context)
{
    int                     ret = onfi_parse_mtdid(context);
    struct onfi_parser_ctx  ctx = {
        .start = context->next
    };

    if (unlikely(ret))
        return ret;

    ret = onfi_parse_parms(&ctx);
    if (unlikely(ret))
        return ret;

    list_entry(onfi_settings.prev,
               struct onfi_setting,
               node)->name = context->start;
    context->next = ctx.next;
    return 0;
}

static
int
onfi_parse_expr(struct onfi_parser_ctx* context)
{
    int ret = onfi_parse_mtd(context);

    if (unlikely(ret))
        return ret;

    if (! *context->next)
        return 0;

    if (unlikely(*context->next != ';'))
        return -EINVAL;

    {
        struct onfi_parser_ctx ctx = {
            .start = context->next + 1
        };

        return onfi_parse_expr(&ctx);
    }
}

static
char const
onfi_signature[] = { 'O', 'N', 'F', 'I' };

#define ONFI_PARM_PAGE_SIZE     256
#define ONFI_PAGE_DATA_SIZE     254
#define ONFI_PAGE_CRC_INIT      ((u16) 0x4F4E)
#define ONFI_NOOP_CMD           -1
#define ONFI_NOOP_ADDR          -1
#define ONFI_PARM_PAGE_NR       4

#define ONFI_PAGE_CRC_OFF       ONFI_PAGE_DATA_SIZE

#define ONFI_OPT_CMD_OFF        8
#define ONFI_OPT_CMD_READCACHE  ((u8) (1 << 1))
#define ONFI_OPT_CMD_CACHEPRG   ((u8) (1 << 0))
#define ONFI_TIMING_MODE_OFF    129

/* FIXME: proper table driven implementation */
static
u16
onfi_crc16_byte(u16 crc, u8 byte)
{
    u8 j;

    for (j = 0x80; j; j >>= 1) {
        u16 bit = crc & 0x8000;

        crc <<= 1;
        if (byte & j)
            bit ^= 0x8000;
        if (bit)
            crc ^= 0x8005;
    }

    return crc;
}

static
u16
onfi_crc16(u16 crc, u8 const* buffer, size_t len)
{
    while (len--)
        crc = onfi_crc16_byte(crc, *buffer++);
    return crc;
}

static
int
onfi_valid_page(char const* parameters, int page_no)
{
    unsigned int        byte, sig_ok;
    char const* const   page = &parameters[page_no * ONFI_PARM_PAGE_SIZE];
    u16                 crc;

    for (byte = 0, sig_ok = 0;
         (byte < sizeof(onfi_signature)) && (sig_ok < 2);
         byte++)
        if (page[byte] == onfi_signature[byte])
            sig_ok++;

    if (unlikely(sig_ok < 2))
        return -1;

    crc = onfi_crc16(ONFI_PAGE_CRC_INIT, page, ONFI_PAGE_DATA_SIZE);
    if (unlikely(crc != *((u16*) &page[ONFI_PAGE_CRC_OFF]))) {
        pr_notice("ONFI: invalid parameter page CRC\n");
        return 0;
    }

    pr_info("ONFI: found valid parameter page %d\n", page_no);
    return 1;
}

#define onfi_merge_timings(setting, timing, name)   \
    ({                                              \
        if (setting->name < 0)                      \
            setting->name = timing->name;           \
     })

static
void
onfi_init_setting(struct onfi_setting* setting, char const* parameters)
{
    struct onfi_timings const*  o_tim;
    struct onfi_timings* const  s_tim = &setting->timings;
    int                         mode = 0;

    if (parameters) {
        setting->read_cache = !! (parameters[ONFI_OPT_CMD_OFF] &
                                  ONFI_OPT_CMD_READCACHE);
        pr_info("ONFI: read cache%ssupported\n",
                (setting->read_cache) ? " " : " not ");
        setting->cache_prg = !! (parameters[ONFI_OPT_CMD_OFF] &
                                 ONFI_OPT_CMD_CACHEPRG);
        pr_info("ONFI: page cache program%ssupported\n",
                (setting->cache_prg) ? " " : " not ");

        pr_info("ONFI: mode %x, feature %x\n", parameters[ONFI_TIMING_MODE_OFF], parameters[8]);
        mode = fls(parameters[ONFI_TIMING_MODE_OFF]);
        if (! mode || mode > array_count(onfi_timings_modes)) {
            pr_warning("ONFI: invalid timing mode found\n");
            mode = 0;
        }
        else {
            mode--;
            setting->read_cache = 0;
            setting->cache_prg = 0;
        }
        setting->feature = !!(parameters[8] & 4);
    }

    pr_info("ONFI: using timings mode %d\n", mode);

    o_tim = &onfi_timings_modes[mode];

	onfi_merge_timings(s_tim, o_tim, tALH);
	onfi_merge_timings(s_tim, o_tim, tALS);
    onfi_merge_timings(s_tim, o_tim, tCEA);
	onfi_merge_timings(s_tim, o_tim, tCEH);
	onfi_merge_timings(s_tim, o_tim, tCH);
	onfi_merge_timings(s_tim, o_tim, tCLH);
	onfi_merge_timings(s_tim, o_tim, tCLR);
	onfi_merge_timings(s_tim, o_tim, tCLS);
	onfi_merge_timings(s_tim, o_tim, tCS);
	onfi_merge_timings(s_tim, o_tim, tDH);
	onfi_merge_timings(s_tim, o_tim, tDS);
	onfi_merge_timings(s_tim, o_tim, tRC);
	onfi_merge_timings(s_tim, o_tim, tREA);
	onfi_merge_timings(s_tim, o_tim, tREH);
	onfi_merge_timings(s_tim, o_tim, tRHZ);
	onfi_merge_timings(s_tim, o_tim, tRP);
	onfi_merge_timings(s_tim, o_tim, tRR);
	onfi_merge_timings(s_tim, o_tim, tWC);
	onfi_merge_timings(s_tim, o_tim, tWH);
	onfi_merge_timings(s_tim, o_tim, tWHR);
	onfi_merge_timings(s_tim, o_tim, tWP);
	onfi_merge_timings(s_tim, o_tim, toRST);
	onfi_merge_timings(s_tim, o_tim, twRST);
	onfi_merge_timings(s_tim, o_tim, teRST);

    if (mode > 2 && s_tim->tRC < 30)
        setting->edo = 1;
    setting->mode = mode;
}

static
int
onfi_read_parms(struct mtd_info* mtd,
                struct onfi_operations const* ops,
                struct onfi_setting* setting)
{
    char    sig[sizeof(onfi_signature)];
    char*   parm_pages;
    int     page = 0;
    int     ret = ops->read_mtd(sig,
                                mtd,
                                ONFI_READID_CMD,
                                ONFI_READID_ADDR,
                                sizeof(onfi_signature));
    if (unlikely(ret)) {
        pr_info("ONFI: failed to read signature for mtd device \"%s\"\n",
                mtd->name);
        return ret;
    }

    if (memcmp(sig, onfi_signature, sizeof(onfi_signature))) {
        pr_info("ONFI: mtd device \"%s\" is not ONFI compliant\n", mtd->name);
        onfi_init_setting(setting, NULL);
        return 0;
    }

    /* 
     * mtd device is ONFI compliant: read parameters page(s) and init settings
     * accordingly
     */
    parm_pages = (char*) __get_free_page(GFP_KERNEL);
    if (! parm_pages)
        return -ENOMEM;

    for (page = 0; page < ONFI_PARM_PAGE_NR; page++) {
        ret = ops->read_mtd(&parm_pages[page * ONFI_PARM_PAGE_SIZE],
                            mtd,
                            (page ? ONFI_NOOP_CMD : ONFI_READPARM_CMD),
                            (page ? ONFI_NOOP_ADDR : ONFI_READPARM_ADDR),
                            ONFI_PARM_PAGE_SIZE);
        if (unlikely(ret)) {
            pr_notice("ONFI: failed to read parameters page %d for mtd device \"%s\"\n",
                      page,
                      mtd->name);
            goto free;
        }

        ret = onfi_valid_page(parm_pages, page);
        if (likely(ret > 0))
            /* valid parameters page */
            break;

        if (ret < 0)
            /* no more parameters page */
            break;

        pr_notice("ONFI: page %d is invalid for mtd device \"%s\"\n",
                  page,
                  mtd->name);
    }

    if (likely(ret > 0))
        onfi_init_setting(setting, &parm_pages[page * ONFI_PARM_PAGE_SIZE]);
    else {
        /* 
         * FIXME: if is possible to rebuild parameters page from other pages
         * using bitwise majority (for example)
         */
        pr_warning("ONFI: no consistent parameters page found\n");
        onfi_init_setting(setting, NULL);
    }

    ret = 0;

free:
    free_page((unsigned long) parm_pages);
    return ret;
}

int
onfi_enable(struct mtd_info* mtd,
            struct onfi_operations const* ops)
{
    struct onfi_setting     setting;
    struct onfi_setting*    set;
    int ret;

    BUG_ON(! mtd);
    BUG_ON(! ops);
    BUG_ON(! ops->init_mtd);
    BUG_ON(! ops->read_mtd);

    if (unlikely(! try_module_get(THIS_MODULE)))
        /* 
         * is this really necessary ? 
         * not handled through dependencies checking ?
         */
        return -EBUSY;

    list_for_each_entry(set, &onfi_settings, node) {
        if (! set->name)
            /* skip default settings */
            continue;

        if (! strcmp(mtd->name, set->name)) {
            memcpy(&setting, set, sizeof(setting));
            goto found;
        }
    }

    memcpy(&setting, onfi_default, sizeof(setting));

found:
    ret = onfi_read_parms(mtd, ops, &setting);
    if (unlikely(ret))
        return ret;

    pr_info("ONFI: timings are set to\n"
        "\ttALH = %d, tALS = %d, tCEA = %d,\n"
        "\ttCEH = %d, tCH = %d, tCLH = %d,\n"
        "\ttCLR = %d, tCLS = %d, tCS = %d,\n"
        "\ttDH = %d, tDS = %d, tRC = %d,\n"
        "\ttREA = %d, tREH = %d, tRHZ = %hd,\n"
        "\ttRP = %d, tRR = %d, tWC = %d,\n"
        "\ttWH = %d, tWHR = %d, tWP = %d,\n"
        "\ttoRST = %hd, twRST = %hd, teRST = %hd\n",
        setting.timings.tALH, setting.timings.tALS, setting.timings.tCEA,
        setting.timings.tCEH, setting.timings.tCH, setting.timings.tCLH,
        setting.timings.tCLR, setting.timings.tCLS, setting.timings.tCS,
        setting.timings.tDH, setting.timings.tDS, setting.timings.tRC,
        setting.timings.tREA, setting.timings.tREH, setting.timings.tRHZ,
        setting.timings.tRP, setting.timings.tRR, setting.timings.tWC,
        setting.timings.tWH, setting.timings.tWHR, setting.timings.tWP,
        setting.timings.toRST, setting.timings.twRST, setting.timings.teRST);

    if (setting.mode) {
        if (setting.feature) {
            char feat[4] = {setting.mode, 0, 0, 0};
            ops->read_mtd(feat,
                    mtd,
                    ONFI_SETFEAT_CMD,
                    1,
                    sizeof(feat));
        }
        else {
            printk("mode!=0, but set feature not supported\n");
        }
    }

    ops->init_mtd(mtd, &setting);
    module_put(THIS_MODULE);

    return 0;
}
EXPORT_SYMBOL(onfi_enable);

static char* parms;
module_param(parms, charp, S_IRUGO);
MODULE_PARM_DESC(parms, "ONFI parmameters list");

static
void
onfi_free(void)
{
    while (! list_empty(&onfi_settings)) {
        struct onfi_setting* setting = list_first_entry(&onfi_settings,
                                                        struct onfi_setting,
                                                        node);
        list_del(&setting->node);
        kfree(setting);
    }
}

static
int __init
onfi_init(void)
{
    int                     ret;
    struct onfi_parser_ctx  context = {
        .start = parms
    };

    if (parms) {
        ret = onfi_parse_expr(&context);
        if (unlikely(ret))
            goto free;
    }

    if (! onfi_default) {
        ret = -ENOMEM;
        onfi_default = onfi_alloc_setting();
        if (unlikely(! onfi_default))
            goto free;
    }

    return 0;

free:
    onfi_free();
    return ret;
}

static
void __exit
onfi_exit(void)
{
    onfi_free();
}

module_init(onfi_init);
module_exit(onfi_exit);

/* vim:ts=4:sw=4:et:syn=c */
