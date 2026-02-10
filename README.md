# **Quad Mod - Quad LFO Module**

### Four LFO channels, 13 waveforms each, internal cross-modulation, in 4 HP

Quad Mod delivers dense modulation in a compact module. Four independent channels each access 13 waveforms—classic shapes, composite waves, and cross-modulated variations.

Adjacent channels can FM or AM modulate each other (4→1→2→3→4), creating evolving polyrhythmic patterns without external patching.
Waveforms include triangle minus sine difference tones (with golden ratio/√3 frequency relationships), frequency-drifting oscillators, random slope, amplitude-breathing textures, and phase-coupled pendulums. 

The cross-modulation routing (FM/AM between neighbors) means simple frequency settings can generate complex evolution as channels influence each other. This way you can create evolving patterns without external patching. 

Useful when you need dense modulation in limited space.
 
 ![quad_mod](https://robertheel.com/content/quad_mod/images/rob_heel_quad_mod_01.jpg) 


# Waveforms

## Classic shapes:

⬤ **TRI**\
![tri](https://robertheel.com/content/quad_mod/images/waveforms/waveform_triangle.svg)\
⬤ **SQR**\
![square](https://robertheel.com/content/quad_mod/images/waveforms/waveform_square.svg)\
⬤ **SIN**\
![sine](https://robertheel.com/content/quad_mod/images/waveforms/waveform_sine.svg)


## Drift & Glitch:

⬤ **Random Slope** 
- Random targets with linear ramps
 ![random_slope](https://robertheel.com/content/quad_mod/images/waveforms/waveform_random_slope.svg) 

⬤ **Wonky TRI** 
- Triangle with frequency drift + glitches.
 ![wonky_tri](https://robertheel.com/content/quad_mod/images/waveforms/waveform_wonky_triangle.svg) 

⬤ **Ratchet TRI** 
- Fast triangle bursts with pauses.
 ![ratchet_tri](https://robertheel.com/content/quad_mod/images/waveforms/waveform_ratchet_triangle.svg) 
<br>

## Cross-Modulation:
⬤ **FM**
- Triangle wave modulated by adjacent LFO.
  - LFO 1 ← modulated by LFO 4
  - LFO 2 ← modulated by LFO 1
  - LFO 3 ← modulated by LFO 2
  - LFO 4 ← modulated by LFO 3
  <br>

⬤ **AM**
- Sine wave amplitude modulated by adjacent LFO (same routing as FM).
# Modulationmatrix

  ![quad_mod_mod_matrix](https://robertheel.com/content/quad_mod/images/quad_mod_modulation_matrix.svg) 
<br>

## Composite/Mathematical:

⬤ **TriSine**
- Triangle minus sine wave, with 4 different frequency ratios that get randomly assigned per channel at startup.
   - 1.618f,  // Golden ratio - naturally pleasing chaos
   - 1.33f,   // 4:3 polyrhythmic - complex but musical
   - 1.7f,    // Prime-ish wonky
   - 1.732f   // √3 - mathematical madness
![trisine](https://robertheel.com/content/quad_mod/images/waveforms/waveform_trisine.svg) 

⬤**SWARM**
- Breathing sine wave with triangle spikes at higher frequencies.
![swarm](https://robertheel.com/content/quad_mod/images/waveforms/waveform_swarm.svg) 

⬤ **PENDULUM**
- Two internal oscillators that drift in and out of phase.
![pendulum](https://robertheel.com/content/quad_mod/images/waveforms/waveform_pendulum.svg) 

## Generative:
⬤ GRAVITY / ELASTIC BAND
- Frequency gets "pulled" toward changing target positions.
![gravity](https://robertheel.com/content/quad_mod/images/waveforms/waveform_gravity_wells.svg) 
<br>
<br>

# Interface
Oled + Encoder to choose waveform for each channel. Left/ Right to change waveforms, encoder click to jump to next channel. Four potentiometers for frequency control of 1, 2, 3, 4. Four outputs. 

# OLED switch-off function
I2C OLED 0.49" 64x32 has a timeout/ switch-off function, as I2C can interfere with fast pwm outputs. Timeout is set to 5 seconds (after last encoder use), but can be easily changed in code.



# Circuit Design
Output stage (1N5819 diode protection, RC low-pass filtering) based on   
[Hagiwos](https://note.com/solder_state/n/nc05d8e8fd311)   Mod1 circuit.


# PIN USAGE - Arduino Nano
Fast PWM OUTPUTS using Timer1 and Timer2:
- Pin 9  (OCR1A) - LFO 1 output
- Pin 10 (OCR1B) - LFO 2 output  
- Pin 11 (OCR2A) - LFO 3 output
- Pin 3  (OCR2B) - LFO 4 output

DIGITAL:
- Pin 2  - Encoder A (INT0 interrupt)
- Pin 4  - Encoder B (regular digital input) 
- Pin 7  - Encoder button/switch

ANALOG INPUTS:
- Pin A0 - LFO 1 frequency pot
- Pin A1 - LFO 2 frequency pot  
- Pin A2 - LFO 3 frequency pot
- Pin A3 - LFO 4 frequency pot

I2C (OLED) 0.49" Screen 64x32 I2C:
- Pin A4 (SDA) - I2C data
- Pin A5 (SCL) - I2C clock

AVAILABLE PINS:
- Pin 5   - Free digital
- Pin 6   - Free digital  
- Pin 12  - Free (MISO)
- Pin 13  - Free (SCK, has onboard LED)
- Pin A6  - Free analog
- Pin A7  - Free analog
<br>
<br>
<br>
# Licenses
Code: MIT License\
Schematic: CC BY-NC 4.0