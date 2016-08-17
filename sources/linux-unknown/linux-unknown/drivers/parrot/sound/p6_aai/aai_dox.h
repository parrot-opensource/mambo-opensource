/**
 * @file   aai_dox.h
 * @brief  AAI doxygen documentation main pages
 *
 * @author gregoire.etienne@parrot.com
 * @date   2008-10-02
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    Parrot Advanced Audio Interface Driver
 */


#ifndef INCONCE_AAI_INTERNAL_DOX_H
#define INCONCE_AAI_INTERNAL_DOX_H

/*! \mainpage Main

<p>

On Linux based platforms, audio streams are used through the Advanced Linux Sound Architecture (ALSA).
As shown below AAI driver which is in <i>kernel space</i> is accessed through ALSA modules (library+driver).
<p>
<div align="center">
<img alt="Audio Layer" src="aai-softlayer.png" border="5">
</div>
<p>
Following chapters will give you more details about ALSA & AAI management:

<ul>
<li>\ref pg_config  : Kernel and Alsa library configuration in lucie.
<li>\ref pg_devices : description and specificities.
<li>\ref pg_ioctl   : description and use.
<li>\ref pg_dma     : description and use.
<li>\ref pg_srconv  : description and use.
<li>\ref pg_utils   : some tools to perform simple tests.
</ul>
<p>
Links to Alsa online documentation:
<ul>
<li><a href="http://www.alsa-project.org/main/index.php/Main_Page"> Alsa project homepage</a>
<li><a href="http://www.alsa-project.org/alsa-doc/alsa-lib/modules.html"> Alsa libary API reference</a>
<li><a href="http://www.alsa-project.org/alsa-doc/alsa-lib/examples.html"> Alsa libary sample codes</a>
</ul>
*/



/*-----------------------------------------------------------------------------
   CONFIGURATION
 *---------------------------------------------------------------------------*/
/*! \page pg_config Lucie Configuration

\section sec_driverconfig AAI driver configuration
If you're making your own kernel configuration, you will have to select the AAI driver.<br>
First, type:
\code
   $make linux-menuconfig
\endcode
then select and configure your AAI driver
\code
   System Type  --->
      Parrot drivers  ---> < > Parrot5+ AAI
                           < > Parrot6 AAI
\endcode
<hr><p>

\section sec_alsaconfig Alsa library configuration
\code
   $make menuconfig
\endcode
then enter <tt>Package Selection for the target</tt> and select an Alsa library release:
<ul>
<li><b>Standard Alsa</b> is the official release(ie the one available with your linux distribution)
this library is quite heavy, binary file is about 1.3 Mbytes
<li><b>Small Alsa</b> is an embedded release, this library binary file is about 200 Kbytes.
</ul>
\code
   Package Selection for the target  --->
      Alsa  --->
        Alsa library release (Alsa standard)  ---> ( ) Alsa standard
                                                   (X) Alsa small(Salsa)
\endcode
<p>

<hr><p>

\section sec_toolsconfig  Audio tools configuration
Some tools can be selected to help test/debug audio.
First, type:
\code
   $make menuconfig
\endcode


<p>
Alsa official utils permits to play or record audio streams(see \ref sec_alsautils for more details).
\code
   Package Selection for the target  --->
      Alsa  --->
        Alsa utils (aplay, arecord, amixer)
\endcode

<p>
AAI tools are designed to test the P6 AAI alsa driver(see \ref sec_aaitools for more details).
\code
   Package Selection for the target  --->
      Alsa  --->
         utils  ---> [*] aai utils
\endcode

*/

/*-----------------------------------------------------------------------------
   DEVICES
 *---------------------------------------------------------------------------*/
/*! \page pg_devices Devices
Here are listed all of the P6 AAI devices
<p>
<h2>Multimedia output devices (i2s)</h2>
AAI can handle 4 speakers through 2 i2s buses (first bus is named "Front", second bus is named "Rear").<br>
Any of these speaker is a mix of 4 sources. AAI driver offers 5 software audio devices described below:<p>
<div align="center">
<table border=2 cellspacing=0 cellpadding=2 style="font-size: 8pt" width=600>
<tr bgcolor=#CCFFCC ><th>Device name	      <th>Description       <th>Dev Nbr<th>Rate          <th>Mode                <th>Outputs</tr>
<tr><td><tt>spk-out0  </tt><td>Speaker music Out0<td>0      <td>[16kHz-48kHz] <td>Interl./Non-interl. <td>Front(left/right)
<tr><td><tt>spk-out1  </tt><td>Speaker music Out1<td>1      <td>[16kHz-48kHz] <td>Interl./Non-interl. <td>Rear(left/right)
<tr><td><tt>spk-8khz  </tt><td>Speaker 8kHz      <td>2      <td> 8kHz         <td>Mono                <td>Front(left/right)-Rear(left/right)
<tr><td><tt>spk-16khz </tt><td>Speaker 16kHz     <td>3      <td> 16kHz        <td>Mono                <td>Front(left/right)-Rear(left/right)
<tr><td><tt>spk-aux   </tt><td>Speaker music Aux <td>4      <td> 44,1kHz/48kHz<td>Interleaved         <td>Front(left/right)-Rear(left/right)
</table></div>
<p>
<b>Note1:</b><br>
For each channel of each speaker device, P6 AAI mixer defines volume and switch controls (see ALSA mixer API for more informations).
<ul>
<li>volume controls of spk-out0 and spk-out1 range is [-63dB;+15dB]
<li>volume controls of spk-8khz, spk-16khz and spk-aux range is [-63dB;0dB]
<li>switch controls permit to mute channels
</ul>
<p>
<div align="center">
<img alt="P6 AAI outputs" src="aai-outputs.png" border="5">
</div>
<p>
<hr>
<p>
<h2>Multimedia input devices (i2s)</h2>
AAI multimedia inputs are componed of 4 micros and 1 feedback devices.<p>
<div align="center">
<table cellspacing=0 cellpadding=2 style="font-size: 8pt" width=600>
<tr bgcolor=#CCFFCC ><th>Device name	      <th>Description       <th>Dev Nbr<th>Rate          <th>Mode       <th>Inputs</tr>
<tr><td><tt>mic0-8khz </tt><td>Micro0 8kHz       <td>5      <td> 8kHz         <td>Interleaved<td>-
<tr><td><tt>mic0-16khz</tt><td>Micro0 16kHz      <td>6      <td> 16kHz        <td>Interleaved<td>-
<tr><td><tt>mic0-music</tt><td>Micro0 music      <td>7      <td> 44,1kHz/48kHz<td>Interleaved<td>-
<tr><td><tt>mic1-music</tt><td>Micro1 music      <td>8      <td> 44,1kHz/48kHz<td>Interleaved<td>-
<tr><td><tt>mic2-8khz </tt><td>Micro2 8kHz       <td>9      <td> 8kHz         <td>Interleaved<td>-
<tr><td><tt>mic2-16khz</tt><td>Micro2 16kHz      <td>10     <td> 16kHz        <td>Interleaved<td>-
<tr><td><tt>mic2-music</tt><td>Micro2 music      <td>11     <td> 44,1kHz/48kHz<td>Interleaved<td>-
<tr><td><tt>mic3-music</tt><td>Micro3 music      <td>12     <td> 44,1kHz/48kHz<td>Interleaved<td>-
<tr><td><tt>back-8khz </tt><td>Feedback 8kHz     <td>13     <td> 8kHz         <td>Interleaved<td>-
<tr><td><tt>back-16khz</tt><td>Feedback 16kHz    <td>14     <td> 16kHz        <td>Interleaved<td>-
<tr><td><tt>back-music</tt><td>Feedback music    <td>15     <td> 44,1kHz/48kHz<td>Interleaved<td>-
</table>
<p>
<img alt="P6 AAI inputs" src="aai-inputs.png">
</div>
<p>
<b>NB:</b>
All of the 3 devices(8kHz,16kHz and music) of Micro0, Micro2 and Feedback, can't be used simultaneously.
<ul>
<li>If music device is playing, 8kHz and 16kHz devices are disabled.
<li>If music is not playing, 8kHz and/or 16kHz devices can be used
</ul>

<hr><p>

<h2>PCM devices</h2>
<table cellspacing=0 cellpadding=2 style="font-size: 8pt" width=600>
<tr bgcolor=#CCFFCC ><th>Device name	      <th>Description<th>Dev Nbr<th>Rate<th>Mode<th>Outputs</tr>
<tr><td><tt>pcm0-out1 </tt><td>PCM0 slot1 out    <td>16     <td> 8kHz/16kHz   <td>Mono<td>-
<tr><td><tt>pcm0-in1  </tt><td>PCM0 slot1 in     <td>17     <td> 8kHz/16kHz   <td>Mono<td>-
<tr><td><tt>pcm0-out2 </tt><td>PCM0 slot2 out    <td>18     <td> 8kHz/16kHz   <td>Mono<td>-
<tr><td><tt>pcm0-in2  </tt><td>PCM0 slot2 in     <td>19     <td> 8kHz/16kHz   <td>Mono<td>-
<tr><td><tt>pcm0-out3 </tt><td>PCM0 slot3 out    <td>20     <td> 8kHz/16kHz   <td>Mono<td>-
<tr><td><tt>pcm0-in3  </tt><td>PCM0 slot3 in     <td>21     <td> 8kHz/16kHz   <td>Mono<td>-
<tr><td><tt>pcm1-out1 </tt><td>PCM1 slot1 out    <td>22     <td> 8kHz/16kHz   <td>Mono<td>-
<tr><td><tt>pcm1-in1  </tt><td>PCM1 slot1 in     <td>23     <td> 8kHz/16kHz   <td>Mono<td>-
<tr><td><tt>pcm1-out2 </tt><td>PCM1 slot2 out    <td>24     <td> 8kHz/16kHz   <td>Mono<td>-
<tr><td><tt>pcm1-in2  </tt><td>PCM1 slot2 in     <td>25     <td> 8kHz/16kHz   <td>Mono<td>-
</table>
*/



/*-----------------------------------------------------------------------------
   IOCTL
 *---------------------------------------------------------------------------*/
/*! \page pg_ioctl Ioctl

Ioctl keys are defined into aai_ioctl.h header file

For a detailled key description, see the \ref aai_hwdep_ioctl documentation

\code
#include <alsa/asoundlib.h>
#include <alsa/hwdep.h>
#include <aai_ioctl.h>
...

void call_ioctl( unsigned int key, unsigned int val )
{
	int i, ret = 0;
	snd_hwdep_t *hwdep;

	if ((ret = snd_hwdep_open(&hwdep, "hw:0,0", O_RDWR)) < 0){
   		fprintf(stdout, "hwdep open failed %d\n", ret);
		exit(1);
	}

	if (snd_hwdep_ioctl(hwdep, key, &val)){
		fprintf(stderr,"unable to access hwdep\n");
		exit(1);
	}
}

...
int main( int argc, char **argv )
{
	uint32_t regval;
	call_ioctl( AAI_IOW_AUXSYNC, AUXSYNC_NONE );
	return 0;
}
\endcode


*/


/*-----------------------------------------------------------------------------
   DMA
 *---------------------------------------------------------------------------*/
/*! \page pg_dma DMA

AAI driver is using DMA transfers. Devices are divided in 3 groups:
<ul>
<li><b>Voice:</b>all devices with a rate of 8 or 16kHz (including PCM devices).
<li><b>Auxiliary:</b> <tt>"mic1-music"</tt> and <tt>"spk-aux"</tt>.
<li><b>Music:</b>all devices with a rate of 44.1 or 48kHz (except <tt>"mic1-music"</tt> and <tt>"spk-aux"</tt>).
</ul>

DMA transfer size is configurable for each group. The table below defines the size of a transfer unit
depending on channel rate and number of streams. Then a DMACNT value (which represents the number of units
of a DMA transfer) is assigned to each group.

<div align="center">
<table border=2 cellspacing=0 cellpadding=2 style="font-size: 8pt" width=600>
<tr><th> <th bgcolor=#CCFFCC >8Khz<th bgcolor=#CCFFCC >16Khz<th bgcolor=#CCFFCC >Music</tr>
<tr><td bgcolor=#CCFFCC ><tt>Mono  </tt> <td>2x32bits   <td>4x32bits   <td>-
<tr><td bgcolor=#CCFFCC ><tt>Stereo</tt> <td>4x32bits   <td>8x32bits   <td>16x32bits
</table></div>
<p>
<p>

\code
Example: Here are some DMA transfer sizes if DMACNT_VOICE is set to 1:
   spk-8khz    --> 1 x (2x32bits) =  8 bytes
   spk-16khz   --> 1 x (4x32bits) = 16 bytes
   mic0-8khz   --> 1 x (4x32bits) = 16 bytes
   mic0-16khz  --> 1 x (8x32bits) = 32 bytes
\endcode
<p>
Each count value can be read and set using ioctl keys.(see \ref pg_ioctl and P6 User manual for more details).

<div align="center">
<table border=2 cellspacing=0 cellpadding=2 style="font-size: 8pt" width=600>
<tr><th> <th bgcolor=#CCFFCC >Max<th bgcolor=#CCFFCC >Read Key<th bgcolor=#CCFFCC >Write Key</tr>
<tr><td bgcolor=#CCFFCC >Voice      <td>8   <td>AAI_IOR_DMACNT_VOICE <td>AAI_IOW_DMACNT_VOICE
<tr><td bgcolor=#CCFFCC >Auxiliary  <td>128 <td>AAI_IOR_DMACNT_AUX   <td>AAI_IOW_DMACNT_AUX
<tr><td bgcolor=#CCFFCC >Music      <td>128 <td>AAI_IOR_DMACNT_MUSIC <td>AAI_IOW_DMACNT_MUSIC
</table></div>
<p>
<b>Note:</b>
<ul>
<li>count value can only be modified if all the devices of the group are closed.
<li>dmacount MUST be set to a power of 2
</ul>

*/



/*-----------------------------------------------------------------------------
   SAMPLE RATE CONVERTER
 *---------------------------------------------------------------------------*/
/*! \page pg_srconv Sample Rate Converter

The hardware Sample Rate Converter can be applied either on inputs(ich2-music and ich3-music),
or on outputs(spk-out0 and spk-out1).
<p>



<h2>SRConv on output devices (spk-out0 and spk-out1)</h2>
Outputs can handle any stream rate as long as the SRConv ratio has been correctly set.
In this case ich0-music, ich2-music and ich3-music devices are disabled.
<div align="center">
<img alt="Sample rate conversion on inputs" src="aai-srconv2.png" border="5">
</div>
<p>

<h2>SRConv on input devices (ich2-music and ich3-music)</h2>
Applying sample rate converter on inputs resynchronizes asynchronous streams
connected to ich2 and ich3 with AC_CLKM.
<ul>
<li>if ich2-music and ich3-music input streams are synchronous with
AC_CLKM and AC_SYNCM, sample rate converter is unused.
</ul>
<ul>
<li>if ich2-music and ich3-music input streams are asynchronous with
AC_CLKM and AC_SYNCM, sample rate converter automatically adapt this
streams for fifos to be synchronous.
</ul>

<div align="center">
<img alt="Sample rate conversion on inputs" src="aai-srconv.png" border="5">
</div>
<p>

<i>NB:Sample Rate Convertor direction can be switched though ioctl call with AAI_IOW_SRCONVOUT key.
Parameter must take following values: SRCONV_IN or SRCONV_OUT. When switching, ich0-music,
ich2-music, ich3-music and spk-outx devices must be closed. </i>


*/




/*-----------------------------------------------------------------------------
   UTILS
 *---------------------------------------------------------------------------*/
/*! \page pg_utils Utils

\section sec_alsautils Alsa utils
To build these tools, see \ref sec_toolsconfig.<br>
These tools are official Alsa utils. Here are some simple commands that can help to perform
some simple tests on audio interfaces.

\subsection ssec_aplay_arecord aplay/arecord
The <b>aplay</b> and <b>arecord</b> applications are for commandline playback and recording of a number of file types including raw, wave and aiff at all the sample rates, bitdepths and channel counts known to the ALSA library.
<p>
Playing a file output(dev0) :
\code
aplay -Dhw:0,0 dtmf_8000_mono.wav
\endcode
<p>
Recording 2 seconds of sound at 44.1kHz, stereo on input(dev1) :
\code
arecord -Dhw:0,7 -d2 -fcd -twav toto.wav
\endcode
<p>
Making a loop from input(dev7) to output(dev0) device
\code
arecord -Dhw:0,7 -fcd - | aplay -Dhw:0,0 -
\endcode
<p>

\subsection ssec_amixer amixer
The <b>amixer</b> application is a command line app which allows adjustments to be made to a devices volume and sound controls.
<p>
<hr>


\section sec_aaitools AAI tools

\subsection ssec_utilsmixer Testing mixer : aai_mixer
Test and use AAI mixer

<p>
\subsection ssec_utilsdevtest Testing Devices : aai_devtest
Test and device's configurations
\code
Usage: aai_devtest <options> [command]

Available options:
  -h,--help        this help
  -v,--verbose     verbose mode
  -l,--list        list devices
  -c,--conflict    test device's conflicts
  -d,--dma         test dma management
  -s,--srconv      test sample rate convertor management
  -a,--all         perform all tests
\endcode

<p>
\subsection ssec_utilsioctl Testing ioctl : aai_ioctl
Test and use ioctl function
\code
Usage: aai_ioctl
 -h             : Display this help
 -v  --verbose  : verbose mode
 -s  --status   : print AAI registers
 -t  --test     : test all ioctl keys

\endcode


*/


/*
! \page pg_concept Conception

\section sec_init Initialization
TODO
\section sec_srconv Sample Rate Converter
A common SRConv is available for ICH2, ICH3, MUSIC_OUT0 and MUSIC_OUT1. In order to avoid
conflicts, SRConv is dedicated to outputs.
<p>
When prepare callback is called rate value is passed to hardware layer which will compute
SR Ratio to store into AAI_SRC_RATIO register.
\code
   ratio = (Fuser / Fi2s) * 2^24
\endcode
\section sec_dma DMA handling

Here is described how AAI DMA should behave:
<h4>open</h4>
<ul>
<li> set correponding bit in AAI_DMACTL register
</ul>
\code
[  ] spk-aux(4) - open
0x00000010: AAI_DMACTL
0x0900f300: AAI_CFG
[II] spk-aux(4) - 48000 Hz, Period: 4096 frames(16384 bytes)
\endcode

<h4>start</h4>
<ul>
<li> Read DMA transfer size
<li> Write DMASA and DMAFA registers depending on DMA transfer size
<li> Enable device interrupt in AAI_ITEN
</ul>
\code
[II] spk-aux(4) - TRIGGER_START
[  ] spk-aux(4) - dmacnt=1 -> 64 bytes
0xffc00000: AAI_AUX_DMASA_OUT
0xffc00040: AAI_AUX_DMAFA_OUT
0x00000010: AAI_ITEN
\endcode

<h4>interrupt</h4>
<ul>
<li> Test if correponding bit in AAI_DMACTL is set
<li> Write next buffer adress in DMAFA register (this will acknowledge IRQ)
</ul>
\code
0xffc00080: AAI_AUX_DMAFA_OUT
\endcode

<h4>stop</h4>
<ul>
<li> Disable device interrupt in AAI_ITEN
</ul>
\code
[II] spk-aux(4) - TRIGGER_STOP
0x00000000: AAI_ITEN
\endcode

<h4>close</h4>
<ul>
<li> Reset correponding bit in AAI_DMACTL register
</ul>
\code
[  ] spk-aux(4) - close
0x00000000: AAI_DMACTL
\endcode

\section sec_concept_volume Volume handling
TODO

\section sec_concept_srconv Sample Rate Convertor
P6 AAI get 1 SRConv for 4 devices (2 inputs and 2 outputs).
TODO

*/





#endif /* CYGONCE_AAI_INTERNAL_DOX_H */
