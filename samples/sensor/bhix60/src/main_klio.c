/*
 * Copyright (c) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdio.h>

/*Define application specific firmware to be uploaded*/
#include <bhix60_klio.h>
#include <firmware/bhi260ap_klio/BHI260AP_KLIO.fw.h>
int bhix60_get_firmware(const struct device *dev, unsigned char **fw,unsigned int *fw_sz)
{
     (void)dev;
     *fw = (unsigned char *)bhy2_firmware_image;
     *fw_sz = sizeof(bhy2_firmware_image);
     return 0;
}

/*KLIO setup*/
int klio_init(const struct device *dev);
/*KLIO clean exit*/
void klio_cleanup(const struct device *dev);
/*force an exit*/
bool exit_condition = false;

void klio_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger);
void main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(bhix60_dev));
	struct sensor_value sv_cfg; 

	struct sensor_trigger klio_trig ={
		.chan = SENSOR_CHAN_BHIX60_KLIO,
		.type =	SENSOR_TRIG_DATA_READY,
	};

	if (!device_is_ready(dev)) {
		printf("Device %s is not ready\n", dev->name);
		return;
	}

	printf("Device %p name is %s\n", dev, dev->name);

	/*Setup KLIO*/
	if(klio_init(dev))
	{
		printf("Unable to initialize KLIO. Exiting.\n");
		return;
	}

	/*Configure KLIO virtual sensor for use*/
	sv_cfg.val1 = 25;       /* Sampling rate in hz */
	sv_cfg.val2 = 0;		 /* Latency */
	sensor_attr_set(dev, SENSOR_CHAN_BHIX60_KLIO,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);
	/*set callback handler for KLIO data*/
	sensor_trigger_set(dev,&klio_trig,klio_trigger_handler);
	while (!exit_condition) {
		/* 40ms period, 25Hz Sampling frequency */
		k_sleep(K_MSEC(40));

    /*If FIFO poll mode is active, fetch needs to be called in 
		a loop to unload events from the BHI60 FIFO loop.*/
#ifdef CONFIG_BHIX60_FIFO_POLL
		sensor_sample_fetch(dev);
#endif

	}
	/*Cleanup before exit, assuming an exit condition is set 
	during processing (currently never set)*/
	klio_cleanup(dev);
}

#define PARAM_BUF_LEN     252

struct klio_runtime
{
    bhy2_klio_sensor_state_t sensor_state;
    uint16_t max_patterns;
    uint16_t max_pattern_size;
    uint8_t ignore_insignificant_movement;
    uint8_t pattern_write_back_index;
    float* similarity_result_buf;
    uint8_t* similarity_idx_buf;
}  klio_rt = {
	.sensor_state = 					/* Will be set by parameter write */
	{
		.learning_enabled = 1, 
		.learning_reset = 1, 
		.recognition_enabled = 1, 
		.recognition_reset = 1
	}, 
	.max_patterns = 0,              	/* Will be retrieved by parameter read */
	.max_pattern_size = 0, 				/* Will be retrieved by parameter read */
	.ignore_insignificant_movement = 1, /* Will be set by parameter write */
	.pattern_write_back_index = 1, 		/* Used by callback routine */
	.similarity_result_buf = NULL, 		/* Will be allocated after we know how many patterns are supported */
	.similarity_idx_buf = NULL 			/* Will be allocated after we know how many patterns are supported */
};

static float parse_klio_handle_learnt_pattern(const struct device *dev,
                                              uint32_t s,
                                              uint32_t ns);
static void print_klio_status(const struct device *dev,int apirslt,char *apiname);

/* Example pattern, BHI260 should be pointing upward, held level, and moved up and down at about 1.5Hz */
uint8_t klio_example_pattern_id = 0;
uint8_t klio_example_pattern[] = {

	0x52, 0x42, 0x31, 0x06, 0x03, 0x8b, 0xff, 0x3c, 0x40, 0x0a, 0xd7, 0x23, 0x3c, 0x73, 0xfe, 0xa7, 0xc0, 0x38,
	0x44, 0xbd, 0x40, 0xbb, 0x1f, 0xe5, 0x3e, 0x38, 0x6f, 0x30, 0x3f, 0x50, 0x89, 0x74, 0x3f, 0x4d, 0x2a, 0xf8,
	0x3c, 0x45, 0x61, 0xd9, 0x40, 0x6d, 0x21, 0x7f, 0x40, 0xd0, 0x80, 0x8f, 0x3d, 0x9e, 0x39, 0x33, 0xbd, 0x51,
	0xc5, 0x0e, 0x3f, 0x64, 0x94, 0x80, 0x3c, 0xba, 0x90, 0xd2, 0x3e, 0xf8, 0xd8, 0x37, 0xbc, 0xed, 0x50, 0xea,
	0x3d, 0xf4, 0x61, 0x16, 0x3f, 0x75, 0xc9, 0x9b, 0xbe, 0x24, 0x20, 0xf7, 0x3c, 0x91, 0x16, 0x5b, 0xbd, 0x0f,
	0x61, 0x21, 0xbc, 0x23, 0xce, 0x80, 0x3e, 0x46, 0x8c, 0x93, 0x3d, 0x0c, 0x70, 0x16, 0x3e, 0x02, 0xf9, 0x9b,
	0x3a, 0x12, 0x48, 0xbc, 0x3d, 0x2e, 0x1f, 0xba, 0x3d, 0xe9, 0x82, 0xf5, 0xbe, 0xb4, 0xbd, 0xe8, 0x3d, 0xc6,
	0x79, 0x02, 0xbd, 0x8a, 0x1a, 0x00, 0x3b, 0x87, 0x22, 0x81, 0x3e, 0x96, 0x57, 0x05, 0x3e, 0xcb, 0x03, 0xcb,
	0xbf, 0x34, 0x2d, 0x93, 0x3e, 0x26, 0x6c, 0xff, 0xbd, 0x52, 0xb0, 0x84, 0x3b
};

/*KLIO data received in FIFO*/
/*NOTE: KLIO data arrives only on-change, so no data will be dumped
unless device is physically moved*/
void klio_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger)
{
	bhy2_klio_sensor_frame_t data_out;
    uint32_t s, ns;
	uint64_t timestamp;
	/*Access KLIO data using KLIO extention API*/
	bhix60_get_klio_frame(dev,&data_out,&timestamp);
	bhix60_convert_time(timestamp, &s, &ns);
    printf("KLIO: T: %d.%09d; Learning [Id:%d Progress:%u Change:%u]; Recognition[Id:%d Count:%d]\n",
           s,
           ns,
           data_out.learn.index,
           data_out.learn.progress,
           data_out.learn.change_reason,
           data_out.recognize.index,
           (int)data_out.recognize.count);
    if (data_out.learn.index != -1) /* -1 means nothing was learnt. */
    {
        float highest_similarity_score = parse_klio_handle_learnt_pattern(dev,s, ns);

        if (highest_similarity_score < 0.6) /* Enable if initial pattern or dissimilar */
        {
            int rc = bhix60_klio_set_pattern_states(dev,KLIO_PATTERN_STATE_ENABLE,
                                                       &klio_rt.pattern_write_back_index,
                                                       1);
            print_klio_status(dev,rc,"bhix60_klio_set_pattern_states");
            klio_rt.pattern_write_back_index++;
        }
    }
}

int klio_init(const struct device *dev)
{
    int8_t rslt;

    /* Get number of supported patterns */
    uint8_t param_buf[PARAM_BUF_LEN];
    uint16_t length = sizeof(param_buf);

    rslt = bhix60_klio_get_parameter(dev,KLIO_PARAM_RECOGNITION_MAX_PATTERNS, param_buf, &length);
    print_klio_status(dev,rslt,"bhix60_klio_get_parameter");
    klio_rt.max_patterns = *((uint16_t*) param_buf);
    klio_rt.similarity_result_buf = k_malloc(sizeof(float) * klio_rt.max_patterns);
    klio_rt.similarity_idx_buf = k_malloc(sizeof(uint8_t) * klio_rt.max_patterns);

    if (klio_rt.similarity_result_buf == NULL || klio_rt.similarity_idx_buf == NULL)
    {
        printf("Unable to allocate Klio buffers.\n");
        return -1;
    }

    /* Get maximum supported pattern size */
    length = sizeof(param_buf);
    rslt = bhix60_klio_get_parameter(dev,KLIO_PARAM_PATTERN_BLOB_SIZE, param_buf, &length);
    print_klio_status(dev,rslt,"bhix60_klio_get_parameter");
    klio_rt.max_pattern_size = *((uint16_t*) param_buf);

    /* Set klio state (learning/recognition enable/disable and reset) */
    rslt = bhix60_klio_set_state(dev,&klio_rt.sensor_state);
    print_klio_status(dev,rslt,"bhix60_klio_set_state");

    /* Prevent learning with small movements, parameter writes should be 
	done after reset and before sensor enable */
    rslt = bhix60_klio_set_parameter(dev,KLIO_PARAM_LEARNING_IGNORE_INSIG_MOVEMENT,
                                   &klio_rt.ignore_insignificant_movement,
                                   sizeof(klio_rt.ignore_insignificant_movement));
    print_klio_status(dev,rslt,"bhix60_klio_set_parameter");

    /* Write example pattern */
    rslt = bhix60_klio_write_pattern(dev,klio_example_pattern_id, klio_example_pattern, sizeof(klio_example_pattern));
    print_klio_status(dev,rslt,"bhix60_klio_write_pattern");

    /*Enables the pattern for recognition, enabling should be done after writing the pattern*/
    rslt = bhix60_klio_set_pattern_states(dev,KLIO_PATTERN_STATE_ENABLE, &klio_example_pattern_id, 1);
    print_klio_status(dev,rslt,"bhy2_klio_set_pattern_states");

    return 0;
}

void klio_cleanup(const struct device *dev)
{
    free(klio_rt.similarity_result_buf);
    free(klio_rt.similarity_idx_buf);
}

static float parse_klio_handle_learnt_pattern(const struct device *dev,
                                              uint32_t s,
                                              uint32_t ns)
{
    uint8_t tmp_buf[PARAM_BUF_LEN];
    uint16_t bufsize = sizeof(tmp_buf);
    float highest_similarity_score = 0.f;

    /* Read out learnt pattern */
    int8_t rslt = bhix60_klio_read_pattern(dev,0, tmp_buf, &bufsize);

    print_klio_status(dev,rslt,"bhix60_klio_read_pattern");

    printf("\r\n");
    printf("KLIO T: %d.%09d; PATTERN LEARNT: ", s, ns);
    for (uint16_t i = 0; i < bufsize; i++)
    {
        printf("%02x", tmp_buf[i]);
    }

    printf("\r\n");

    /* Write back learnt pattern for recognition */
    if (klio_rt.pattern_write_back_index < klio_rt.max_patterns)
    {
        /* Write pattern for recognition, note that this resets recognition statistics (and repetition counts) */
        rslt = bhix60_klio_write_pattern(dev,klio_rt.pattern_write_back_index, tmp_buf, bufsize);
        print_klio_status(dev,rslt,"bhix60_klio_write_pattern");

        if (klio_rt.pattern_write_back_index > 0)
        {
            /* Compare current pattern against all previously stored ones */
            for (uint8_t i = 0; i < klio_rt.pattern_write_back_index; i++)
            {
                klio_rt.similarity_idx_buf[i] = i;
            }

            printf("\n");
            rslt = bhix60_klio_similarity_score_multiple(dev,
													klio_rt.pattern_write_back_index,
                                                    klio_rt.similarity_idx_buf,
                                                    klio_rt.pattern_write_back_index,
                                                    klio_rt.similarity_result_buf);
            print_klio_status(dev,rslt,"bhix60_klio_similarity_score_multiple");
            printf("KLIO T: %d.%09d; SIMILARITY SCORE TO ALREADY STORED PATTERNS: ",
                   s,
                   ns);
            for (uint8_t i = 0; i < klio_rt.pattern_write_back_index; i++)
            {
                float tmp_score = klio_rt.similarity_result_buf[i];

                printf("%d: %d ", i, (int)tmp_score);

                if (tmp_score > highest_similarity_score)
                {
                    highest_similarity_score = tmp_score;
                }
            }

            printf("\r\n");
        }
    }

    /* If we have no stored patterns return will be 0.f */

    return highest_similarity_score;
}

static void print_klio_status(const struct device *dev,int apirslt,char *apiname)
{
    uint32_t klio_status;
	if(apirslt)
		printf("Error calling:%s\n",apiname);		
    int8_t rslt = bhix60_klio_read_reset_driver_status(dev,&klio_status);
    if(rslt)
		printf("Error getting KLIO status\n");
	else
		printf("KLIO status: %d (%s)\n",klio_status,apiname);
}

