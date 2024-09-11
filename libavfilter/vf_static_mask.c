// Include necessary headers
#include "libavutil/dict.h"
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavutil/eval.h"
#include "libavutil/pixdesc.h"
#include "libavutil/motion_vector.h"
#include "libavutil/video_enc_params.h"
#include "libavformat/url.h"
#include "libavformat/http.h"
#include "libavformat/avio.h"
#include "avfilter.h"
#include "formats.h"
#include "drawutils.h"
#include "libavutil/internal.h"
#include "video.h"
#include "libavutil/mem.h" // Add this include for memory allocation functions

struct static_maskContext;
typedef int (*PixelBelongsToRegion)(struct static_maskContext *s, int x, int y);

typedef struct static_maskContext {
    const AVClass *class;
    int size;               // Chunk size
    double threshold;       // Difference threshold
    int frame_back;         // Number of frames to buffer and look back
    uint64_t **prev_sums;   // Buffer to store pixel sums of previous frames
    int prev_width;         // Width of the frame for previous sums
    int prev_height;        // Height of the frame for previous sums
    int frame_count;        // Counter to track the current frame number

} static_maskContext;

#define OFFSET(x) offsetof(static_maskContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM|AVFILTER_FLAG_DYNAMIC_OUTPUTS

static const AVOption static_mask_options[] = {
    {"size", "The side of the rectangle", OFFSET(size), AV_OPT_TYPE_INT, {.i64 = 20}, 0, 600, FLAGS },
    {"threshold", "Difference threshold", OFFSET(threshold), AV_OPT_TYPE_DOUBLE, {.dbl = 20}, 0, 1000, FLAGS },
    {"frame_back", "Number of frames to buffer and compare", OFFSET(frame_back), AV_OPT_TYPE_INT, {.i64 = 1}, 1, 100, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(static_mask);

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_YUV444P,     AV_PIX_FMT_YUV422P,  AV_PIX_FMT_YUV420P,
    AV_PIX_FMT_YUV411P,     AV_PIX_FMT_YUV410P,
    AV_PIX_FMT_YUVJ444P,    AV_PIX_FMT_YUVJ422P, AV_PIX_FMT_YUVJ420P,
    AV_PIX_FMT_YUV440P,     AV_PIX_FMT_YUVJ440P,
    AV_PIX_FMT_YUVA420P,    AV_PIX_FMT_YUVA422P, AV_PIX_FMT_YUVA444P,
    AV_PIX_FMT_RGB24,       AV_PIX_FMT_BGR24,
    AV_PIX_FMT_RGBA,        AV_PIX_FMT_BGRA,
    AV_PIX_FMT_ARGB,        AV_PIX_FMT_ABGR,
    AV_PIX_FMT_0RGB,        AV_PIX_FMT_0BGR,
    AV_PIX_FMT_RGB0,        AV_PIX_FMT_BGR0,
    AV_PIX_FMT_NONE
};

static int filter_frame(AVFilterLink *inlink, AVFrame *frame) {
    AVFilterContext *ctx = inlink->dst;
    static_maskContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    int x, y, i, j;
    int range_counter = 0;

    // Get the frame index in the buffer based on the frame_back parameter
    int buffer_index = s->frame_count % s->frame_back;

    for (y = 0; y < frame->height; y += s->size) {
        for (x = 0; x < frame->width; x += s->size) {
            uint64_t sum = 0;

            for (j = y; j < (y + s->size) && j < frame->height; ++j) {
                for (i = x; i < (x + s->size) && i < frame->width; ++i) {
                    sum += frame->data[0][j * frame->linesize[0] + i];  // Y component
                    sum += frame->data[1][j/2 * frame->linesize[1] + i/2];  // U component
                    sum += frame->data[2][j/2 * frame->linesize[2] + i/2];  // V component
                }
            }

            // Compare with the previous frame at the specified buffer index
            if ((abs(sum - s->prev_sums[buffer_index][range_counter]) / (s->size * s->size / 10)) < s->threshold) {
                for (j = y; j < y + s->size && j < frame->height; ++j) {
                    for (i = x; i < x + s->size && i < frame->width; ++i) {
                        frame->data[0][j * frame->linesize[0] + i] = 16;
                        frame->data[1][j/2 * frame->linesize[1] + i/2] = 128;
                        frame->data[2][j/2 * frame->linesize[2] + i/2] = 128;
                    }
                }
            }

            // Store the current sum into the buffer
            s->prev_sums[buffer_index][range_counter] = sum;
            range_counter++;
        }
    }

    // Increment the frame count
    s->frame_count++;

    return ff_filter_frame(outlink, frame);
}

static int config_input(AVFilterLink *inlink) {
    static_maskContext *s = inlink->dst->priv;
    s->prev_width = (inlink->w + s->size - 1) / s->size; // Round up division
    s->prev_height = (inlink->h + s->size - 1) / s->size; // Round up division

    // Allocate buffer for previous sums based on frame_back
    s->prev_sums = av_malloc_array(s->frame_back, sizeof(*s->prev_sums));
    if (!s->prev_sums)
        return AVERROR(ENOMEM);

    for (int i = 0; i < s->frame_back; i++) {
        s->prev_sums[i] = av_mallocz(s->prev_width * s->prev_height * sizeof(*s->prev_sums[i]));
        if (!s->prev_sums[i])
            return AVERROR(ENOMEM);
    }

    s->frame_count = 0; // Initialize frame counter

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx) {
    static_maskContext *s = ctx->priv;
    if (s->prev_sums) {
        for (int i = 0; i < s->frame_back; i++) {
            av_free(s->prev_sums[i]);
        }
        av_free(s->prev_sums);
    }
}

static const AVFilterPad static_mask_inputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
        .config_props = config_input,
        .flags = AVFILTERPAD_FLAG_NEEDS_WRITABLE,
    },
};

static const AVFilterPad static_mask_outputs[] = { 
    { .name = "default", .type = AVMEDIA_TYPE_VIDEO, }, 
};

const AVFilter ff_vf_static_mask = {
    .name           = "static_mask",
    .description    = NULL_IF_CONFIG_SMALL("Mask static image areas"),
    .priv_size      = sizeof(static_maskContext),
    .priv_class     = &static_mask_class,
    .flags          = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
    .uninit         = uninit, // Add uninit to free the allocated memory
    FILTER_INPUTS(static_mask_inputs),
    FILTER_OUTPUTS(static_mask_outputs),
    FILTER_PIXFMTS_ARRAY(pix_fmts),
};
