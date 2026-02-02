import serial
import time
import threading
import random
from flask import Flask, request
from flask_cors import CORS
import os
import subprocess

app = Flask(__name__)
CORS(app)

# Serial connection to Arduino
ser = serial.Serial('/dev/serial0', 100000, timeout=1)
last_command_time = time.time()
RETURN_TO_CENTER_DELAY = 5
IDLE_TIMEOUT = 3  # Reduced for more frequent idles during testing
IDLE_INTERVAL_MIN = 1
IDLE_INTERVAL_MAX = 2  # Shorter max sleep for more frequent movement
SOUND_FOLDER = 'hanus_sound'  # folder for sound clips in same directory as script
SOUND_INTERVAL = 60
last_sound_time = 0

# Global flag to pause breathing during other actions
is_performing_action = False

# Filenames that trigger the thoughtful movement – adapt to hexapod gentle sway or tilt
THOUGHTFUL_QUOTES = {
    'skinnyhuman.wav',
    'skinnyhuman2.wav',
    'skinnyhuman3.wav',
    'lonely.wav',
    'wisdom.wav',
    'asitshouldbe.wav',
    'universe.wav',
    'permanent.wav',
    'curious.wav',
    'rest.wav',
}


#Always in Translate mode when Pi in control
#Only use ch1-7 for idles- everything else stays 0

#ch1  Left forward/back (128 neutral, >128 forward, <128 back)
#ch2  Left left/right (128 neutral, >128 right, <128 left)
#ch3  Left twist (unused in translate mode)
#ch4  Right up/down (pitch: 128 neutral, >128 forward/down tilt, <128 backward/up tilt)
#ch5  Right left/right (roll: 128 neutral, >128 right roll, <128 left roll)
#ch6  Right twist (torso yaw: 128 neutral, >128 right yaw, <128 left yaw)
#ch7  Body Height (128 mid, >128 higher, <128 lower)
#ch8  Gait Speed (always 0)
#ch9  Input_source (always 0)
#ch10 Mode Select (always 0 for translate mode)
#ch11 Single leg momentary (always 0)
#ch12 Gait cycle momentary (always 0)
#ch13 Balance Mode (always 0 for now, not implemented yet)
#ch14 Walk Method (always 0)
#ch15 Double Travel (always 0)

# Mood-to-SBUS string dict – tuned for a deep thinker: slow, subtle, contemplative movements
# Values are gentle and minimal to reflect quiet wisdom rather than depression
MOODS = {
    'neutral': '128,128,128,128,128,128,128,0,255,0,0,0,0,0,0',  # Balanced, mid height
    'sadness': '128,128,128,135,128,128,120,0,255,0,0,0,0,0,0',  # Slight forward tilt (introspective), slightly lower height
    'joy': '128,128,128,120,128,128,140,0,255,0,0,0,0,0,0',      # Gentle backward tilt (uplifted), slightly higher height
    'anger': '130,128,128,130,128,128,128,0,255,0,0,0,0,0,0',    # Subtle forward shift, forward tilt, neutral height
    'fear': '126,128,128,128,128,128,120,0,255,0,0,0,0,0,0',     # Slight backward shift, lower height
    'surprise': '128,128,128,128,128,135,130,0,255,0,0,0,0,0,0', # Subtle yaw, slightly higher height
    'disgust': '128,126,128,128,126,128,128,0,255,0,0,0,0,0,0',  # Gentle left shift and roll, neutral height
    'default': '128,128,128,128,128,128,128,0,255,0,0,0,0,0,0'   # Same as neutral
}

# Set volume at start
subprocess.call(['amixer', 'sset', 'Playback', '70%'])

def send_sbus_string(sbus_str):
    command = (sbus_str + '\n').encode()
    ser.write(command)
    ser.flush()

def return_to_neutral():
    print("Returning to neutral after delay")
    send_sbus_string(MOODS['neutral'])
    global last_command_time
    last_command_time = time.time()

def perform_thoughtful_movement():
    global is_performing_action
    is_performing_action = True
    print("Hanuš is deep in thought - gentle sway or tilt")
   
    # Send a "thoughtful" SBUS string
    thoughtful_str = '128,128,128,132,128,128,125,0,255,0,0,0,0,0,0' # Slight forward tilt, subtle lower height for contemplation
    send_sbus_string(thoughtful_str)
    time.sleep(2.0) # Slower pause to "ponder"
   
    # Gentle "sway" sequence – slow, minimal side-to-side or up/down
    sway_sequences = [
       '128,130,128,128,130,128,125,0,255,0,0,0,0,0,0', # Slight right shift and roll
       '128,126,128,128,126,128,125,0,255,0,0,0,0,0,0', # Slight left shift and roll
       '128,128,128,128,128,128,125,0,255,0,0,0,0,0,0' # Back to center
    ]
    for sway_str in sway_sequences:
        send_sbus_string(sway_str)
        time.sleep(2.0) # Slower timing to match thoughtful voice lines
   
    print("Thoughtful movement complete")
    is_performing_action = False

@app.route('/move', methods=['GET'])
def move():
    global last_command_time
    emotion = request.args.get('emotion', 'neutral').lower()
    print(f"Received move command for emotion: {emotion}")
   
    sbus_str = MOODS.get(emotion, MOODS['default'])
    send_sbus_string(sbus_str)
   
    last_command_time = time.time()
    threading.Timer(RETURN_TO_CENTER_DELAY, return_to_neutral).start()
    return 'Movement executed'

def parse_sbus_string(sbus_str):
    return list(map(int, sbus_str.split(',')))

def generate_sbus_string(values):
    return ','.join(map(str, values))

def ramp_to(start_str, end_str, steps=5, delay=0.2):
    start_values = parse_sbus_string(start_str)
    end_values = parse_sbus_string(end_str)
    for i in range(1, steps + 1):
        interp_values = [int(start + (end - start) * i / steps) for start, end in zip(start_values, end_values)]
        interp_str = generate_sbus_string(interp_values)
        send_sbus_string(interp_str)
        time.sleep(delay)

def gentle_idles_thread():
    # Define base neutral
    neutral = MOODS['neutral']
   
    # Define gentle sequences with ramp targets for swaying, breathing, bouncing
    gentle_sequences = [
        # Gentle side sway (left-right roll/shift)
        '128,138,128,128,138,128,128,0,255,0,0,0,0,0,0', # Right shift/roll
        '128,118,128,128,118,128,128,0,255,0,0,0,0,0,0', # Left shift/roll
        neutral, # Center
       
        # Gentle up/down breathing/bounce (height)
        '128,128,128,128,128,128,138,0,255,0,0,0,0,0,0', # Higher
        '128,128,128,128,128,128,118,0,255,0,0,0,0,0,0', # Lower
        neutral, # Neutral
       
        # Subtle forward/back (tilt/shift)
        '138,128,128,138,128,128,128,0,255,0,0,0,0,0,0', # Forward
        '118,128,128,118,128,128,128,0,255,0,0,0,0,0,0', # Backward
        neutral, # Neutral
       
        # Gentle yaw (left-right twist)
        '128,128,128,128,128,138,128,0,255,0,0,0,0,0,0', # Right yaw
        '128,128,128,128,128,118,128,0,255,0,0,0,0,0,0', # Left yaw
        neutral # Center
    ]
   
    current_pos = neutral
    while True:
        if not is_performing_action:
            # Pick a random gentle sequence target
            target = random.choice(gentle_sequences)
            # Ramp to it smoothly
            ramp_to(current_pos, target, steps=10, delay=0.1) # 10 steps, 0.1s each for smooth ~1s transition
            current_pos = target
            time.sleep(1.0) # Hold briefly before next
        else:
            time.sleep(0.1) # Check frequently when paused

def idle_thread():
    global last_sound_time, is_performing_action
    while True:
        current_time = time.time()
        time_since_last = current_time - last_command_time
        if time_since_last > IDLE_TIMEOUT:
            print("Idle detected, performing random movement")
            is_performing_action = True

            # Idle SBUS actions – separated into tiny twitches (constant small changes) and occasional bigger sequences
            # Tiny twitches: very small adjustments for lifelike constant motion
            tiny_twitches = [
                '158,128,128,128,128,128,128,0,255,0,0,0,0,0,0',  # Tiny forward
                '98,128,128,128,128,128,128,0,255,0,0,0,0,0,0',  # Tiny backward
                '128,158,128,128,128,128,128,0,255,0,0,0,0,0,0',  # Tiny right shift
                '128,98,128,128,128,128,128,0,255,0,0,0,0,0,0',  # Tiny left shift
                '128,128,128,158,128,128,128,0,255,0,0,0,0,0,0',  # Tiny forward tilt
                '128,128,128,98,128,128,128,0,255,0,0,0,0,0,0',  # Tiny backward tilt
                '128,128,128,128,158,128,128,0,255,0,0,0,0,0,0',  # Tiny right roll
                '128,128,128,128,98,128,128,0,255,0,0,0,0,0,0',  # Tiny left roll
                '128,128,128,128,128,158,128,0,255,0,0,0,0,0,0',  # Tiny right yaw
                '128,128,128,128,128,98,128,0,255,0,0,0,0,0,0',  # Tiny left yaw
                '128,128,128,128,128,128,158,0,255,0,0,0,0,0,0',  # Tiny height up
                '128,128,128,128,128,128,98,0,255,0,0,0,0,0,0'   # Tiny height down
            ]

            # Bigger idle actions: occasional sequences or singles for variety
            bigger_idles = [
                # Single subtle idles
                '139,128,128,139,128,128,140,0,255,0,0,0,0,0,0',  # Tiny forward shift and tilt, slight higher height
                '128,139,128,128,139,128,116,0,255,0,0,0,0,0,0',  # Tiny right shift and roll, slight lower height
                '117,128,128,117,128,139,128,0,255,0,0,0,0,0,0',  # Tiny backward shift, backward tilt, slight yaw
                '128,117,128,128,117,128,140,0,255,0,0,0,0,0,0',  # Tiny left shift, negative roll, slight higher height
                '128,128,128,140,128,140,115,0,255,0,0,0,0,0,0',  # Subtle forward tilt and yaw, slight lower height
                '128,128,128,128,140,117,139,0,255,0,0,0,0,0,0',  # Subtle positive roll, negative yaw, slight higher height
                '128,128,128,116,117,128,116,0,255,0,0,0,0,0,0',  # Subtle backward tilt and roll, slight lower height

                # Sequences for bigger movements (looping idles)
                [  # Gentle side sway loop
                    '128,139,128,128,139,128,128,0,255,0,0,0,0,0,0',  # Right shift/roll
                    '128,117,128,128,117,128,128,0,255,0,0,0,0,0,0',  # Left shift/roll
                    '128,128,128,128,128,128,128,0,255,0,0,0,0,0,0'   # Center
                ],
                [  # Subtle up/down "breathing" loop
                    '128,128,128,128,128,128,142,0,255,0,0,0,0,0,0',  # Slightly higher
                    '128,128,128,128,128,128,114,0,255,0,0,0,0,0,0',  # Slightly lower
                    '128,128,128,128,128,128,128,0,255,0,0,0,0,0,0'   # Neutral
                ],
                [  # Slow forward/back contemplation loop
                    '139,128,128,139,128,128,128,0,255,0,0,0,0,0,0',  # Tiny forward
                    '117,128,128,117,128,128,128,0,255,0,0,0,0,0,0',  # Tiny backward
                    '128,128,128,128,128,128,128,0,255,0,0,0,0,0,0'   # Neutral
                ],
                [  # Gentle yaw scan loop (like pondering surroundings)
                    '128,128,128,128,128,140,128,0,255,0,0,0,0,0,0',  # Slight right yaw
                    '128,128,128,128,128,116,128,0,255,0,0,0,0,0,0',  # Slight left yaw
                    '128,128,128,128,128,128,128,0,255,0,0,0,0,0,0',  # Center
                    '128,128,128,128,128,128,128,0,255,0,0,0,0,0,0'   # Hold center briefly
                ]
            ]

            # Favor bigger idles most of the time (80% chance), tiny twitches occasionally (20%)
            if random.random() < 0.2:
                action = random.choice(tiny_twitches)
                print(f"Selected tiny twitch: {action}")
                send_sbus_string(action)
            else:
                action = random.choice(bigger_idles)
                if isinstance(action, str):
                    # Single bigger idle
                    print(f"Selected single bigger idle string: {action}")
                    send_sbus_string(action)
                else:
                    # Looping sequence: send each with delay, repeat 1 time for subtlety
                    repeat = 1  # Fixed to 1 for less repetition
                    print(f"Selected looping idle sequence (repeat {repeat}x): {action}")
                    for _ in range(repeat):
                        for idle_str in action:
                            send_sbus_string(idle_str)
                            time.sleep(2.0)  # Slower delay for mopey, thoughtful pacing
            
            print("Idle movement completed")

            if current_time - last_sound_time > SOUND_INTERVAL:
                wav_files = [f for f in os.listdir(SOUND_FOLDER) if f.endswith('.wav')]
                if wav_files:
                    random_wav = random.choice(wav_files)
                    full_path = os.path.join(SOUND_FOLDER, random_wav)
                    print(f"Playing sound: {random_wav}")

                    is_thoughtful_quote = random_wav in THOUGHTFUL_QUOTES

                    player = subprocess.Popen(['aplay', full_path])

                    if is_thoughtful_quote:
                        threading.Thread(target=perform_thoughtful_movement, daemon=True).start()

                    player.wait()

                    last_sound_time = current_time
            is_performing_action = False
        else:
            print("Not idle yet")
        sleep_duration = random.randint(IDLE_INTERVAL_MIN, IDLE_INTERVAL_MAX)
        print("Sleeping for {} seconds".format(sleep_duration))
        time.sleep(sleep_duration)

if __name__ == '__main__':
    print("Starting gentle idles thread")
    threading.Thread(target=gentle_idles_thread, daemon=True).start()
    print("Gentle idles thread started")
    print("Starting idle thread")
    threading.Thread(target=idle_thread, daemon=True).start()
    print("Idle thread started, launching Flask app")
    app.run(host='0.0.0.0', port=5004)
