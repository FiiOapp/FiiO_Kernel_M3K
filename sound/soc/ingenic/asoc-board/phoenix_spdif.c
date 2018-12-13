 /*
 * Copyright (C) 2014 Ingenic Semiconductor Co., Ltd.
 *	http://www.ingenic.com
 * Author: cli <chen.li@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/gpio.h>

static struct snd_soc_ops phoenix_i2s_ops = {

};

static struct snd_soc_dai_link phoenix_dais[] = {
	[0] = {
		.name = "PHOENIX ICDC",
		.stream_name = "PHOENIX ICDC",
		.platform_name = "jz-asoc-aic-dma",
		.cpu_dai_name = "jz-asoc-aic-spdif",
		.codec_dai_name = "spdif dump dai",
		.codec_name = "spdif dump",
		.ops = &phoenix_i2s_ops,
	},
	[1] = {
		.name = "PHOENIX PCMBT",
		.stream_name = "PHOENIX PCMBT",
		.platform_name = "jz-asoc-pcm-dma",
		.cpu_dai_name = "jz-asoc-pcm",
		.codec_dai_name = "pcm dump dai",
		.codec_name = "pcm dump",
	},
	[2] = {
		.name = "PHOENIX DMIC",
		.stream_name = "PHOENIX DMIC",
		.platform_name = "jz-asoc-dmic-dma",
		.cpu_dai_name = "jz-asoc-dmic",
		.codec_dai_name = "dmic dump dai",
		.codec_name = "dmic dump",
	},
};

static struct snd_soc_card phoenix = {
	.name = "phoenix",
	.owner = THIS_MODULE,
	.dai_link = phoenix_dais,
	.num_links = ARRAY_SIZE(phoenix_dais),
};

static int snd_phoenix_probe(struct platform_device *pdev)
{
	int ret = 0;
	phoenix.dev = &pdev->dev;
	ret = snd_soc_register_card(&phoenix);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card failed %d\n", ret);
	return ret;
}

static int snd_phoenix_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&phoenix);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver snd_phoenix_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ingenic-alsa",
		.pm = &snd_soc_pm_ops,
	},
	.probe = snd_phoenix_probe,
	.remove = snd_phoenix_remove,
};
module_platform_driver(snd_phoenix_driver);

MODULE_AUTHOR("sccheng<shicheng.cheng@ingenic.com>");
MODULE_DESCRIPTION("ALSA SoC phoenix Snd Card");
MODULE_LICENSE("GPL");
