/*
 * sec_ts_interface.c - samsung ts driver custom interface
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include "include/sec_ts.h"
#include "linux/device.h"

static void sec_ts_lowpowermode_set(int state)
{
	struct sec_ts_data *ts = get_sec_ts_data();

	if (!ts) {
		pr_err("ERR: %s: Not able to get sec_ts_data!\n", __func__);
		return;
	}

	mutex_lock(&ts->aod_mutex);

	input_info(true, &ts->client->dev, "Current power status: %d\n",
		   ts->power_status);

	if (sec_ts_get_pw_status() || !ts->after_work.done ||
	    (ts->power_status == SEC_TS_STATE_POWER_OFF)) {
		ts->aod_pending = true;
		ts->aod_pending_lowpower_mode = state;
		input_info(true, &ts->client->dev,
			   "Postponing lowpower_mode: %d\n",
			   ts->aod_pending_lowpower_mode);
	} else if (ts->power_status == SEC_TS_STATE_LPM &&
		   state == TO_LOWPOWER_MODE) {
		ts->aod_pending = true;
		ts->aod_pending_lowpower_mode = state;
		input_info(true, &ts->client->dev,
			   "Postponing lowpower_mode: %d\n",
			   ts->aod_pending_lowpower_mode);
	} else {
		/* set lowpower mode by spay, edge_swipe function. */
		ts->lowpower_mode = state;
		sec_ts_set_lowpowermode(ts, ts->lowpower_mode);
	}

exit:
	mutex_unlock(&ts->aod_mutex);
	return;
}

void sec_ts_lpmode_enable(void)
{
	sec_ts_lowpowermode_set(TO_LOWPOWER_MODE);
	pr_info("Enabling lowpower mode by external interface\n");
}
EXPORT_SYMBOL(sec_ts_lpmode_enable);

void sec_ts_lpmode_disable(void)
{
	sec_ts_lowpowermode_set(TO_TOUCH_MODE);
	pr_info("Disabling lowpower mode by external interface\n");
}
EXPORT_SYMBOL(sec_ts_lpmode_disable);
