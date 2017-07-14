#!/bin/bash
# Kernel Build Script

BUILD_KERNEL_DIR=$(pwd)
BUILD_KERNEL_OUT=$BUILD_KERNEL_DIR/../dreamq_kernel_out
BUILD_KERNEL_OUT_DIR=$BUILD_KERNEL_OUT/KERNEL_OBJ

BUILD_CROSS_COMPILE=$C_KERNEL_TOOLCHAIN
BUILD_JOB_NUMBER=`grep processor /proc/cpuinfo|wc -l`

KERNEL_DEFCONFIG=$1_chn_open_defconfig

KERNEL_IMG_NAME=Image.gz-dtb
KERNEL_IMG=$BUILD_KERNEL_OUT/$KERNEL_IMG_NAME

FUNC_GENERATE_DEFCONFIG()
{
	echo ""
        echo "=============================================="
        echo "START : FUNC_GENERATE_DEFCONFIG"
        echo "=============================================="
        echo "build config="$KERNEL_DEFCONFIG ""
        echo ""

	make -C $BUILD_KERNEL_DIR O=$BUILD_KERNEL_OUT_DIR -j$BUILD_JOB_NUMBER ARCH=arm64 \
			CROSS_COMPILE=$BUILD_CROSS_COMPILE \
			$KERNEL_DEFCONFIG || exit -1

	cp $BUILD_KERNEL_OUT_DIR/.config $BUILD_KERNEL_DIR/arch/arm64/configs/$KERNEL_DEFCONFIG

	echo ""
	echo "================================="
	echo "END   : FUNC_GENERATE_DEFCONFIG"
	echo "================================="
	echo ""
}

FUNC_BUILD_KERNEL()
{
	echo ""
	echo "================================="
	echo "START   : FUNC_BUILD_KERNEL"
	echo "================================="
	echo ""
	rm $KERNEL_IMG $BUILD_KERNEL_OUT_DIR/arch/arm64/boot/Image
	rm -rf $BUILD_KERNEL_OUT_DIR/arch/arm64/boot/dts

if [ "$USE_CCACHE" ]
then
	make -C $BUILD_KERNEL_DIR O=$BUILD_KERNEL_OUT_DIR -j$BUILD_JOB_NUMBER ARCH=arm64 \
			CROSS_COMPILE=$BUILD_CROSS_COMPILE \
			CC="ccache "$BUILD_CROSS_COMPILE"gcc" CPP="ccache "$BUILD_CROSS_COMPILE"gcc -E" || exit -1
else
	make -C $BUILD_KERNEL_DIR O=$BUILD_KERNEL_OUT_DIR -j$BUILD_JOB_NUMBER ARCH=arm64 \
			CROSS_COMPILE=$BUILD_CROSS_COMPILE || exit -1
fi

	cp $BUILD_KERNEL_OUT_DIR/arch/arm64/boot/$KERNEL_IMG_NAME $KERNEL_IMG
	echo "Made Kernel image: $KERNEL_IMG"
	echo "================================="
	echo "END   : FUNC_BUILD_KERNEL"
	echo "================================="
	echo ""
}

(
    START_TIME=`date +%s`

    FUNC_GENERATE_DEFCONFIG
    FUNC_BUILD_KERNEL

    END_TIME=`date +%s`

    let "ELAPSED_TIME=$END_TIME-$START_TIME"
    echo "Total compile time is $ELAPSED_TIME seconds"
) 2>&1
