import argparse
import math
import os
import random
import struct
import subprocess
import sys
import wave

SAMPLE_RATE = 44100

def clamp_sample(value: float) -> int:
    value = max(-1.0, min(1.0, value))
    return int(value * 32767)

def make_bark_waveform(sample_rate: int = SAMPLE_RATE) -> list[int]:
    random.seed(5)
    samples: list[int] = []
    bark_pattern = [
        (0.00, 0.16, 190, 130, 0.95),
        (0.18, 0.14, 170, 110, 0.85),
        (0.43, 0.18, 210, 140, 1.00),
        (0.64, 0.12, 160, 95, 0.70),
    ]
    total_samples = int(1.0 * sample_rate)

    for i in range(total_samples):
        t = i / sample_rate
        sample = 0.0
        for start, duration, f0, f1, gain in bark_pattern:
            if start <= t < start + duration:
                local_t = (t - start) / duration
                freq = f0 + (f1 - f0) * local_t
                attack = min(1.0, local_t / 0.08)
                decay = max(0.0, 1.0 - local_t)
                envelope = attack * (decay ** 0.6)
                tone = (
                    math.sin(2 * math.pi * freq * t)
                    + 0.45 * math.sin(2 * math.pi * freq * 2.1 * t + 0.3)
                    + 0.2 * math.sin(2 * math.pi * freq * 3.2 * t + 0.1)
                ) / 1.65
                noise = random.uniform(-1.0, 1.0) * 0.35 * (1.0 - local_t * 0.7)
                sample += gain * envelope * (0.72 * tone + 0.28 * noise)
        samples.append(clamp_sample(sample * 0.85))

    return samples

def write_wave(path: str, samples: list[int], sample_rate: int = SAMPLE_RATE) -> None:
    with wave.open(path, "wb") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate)
        frame_data = b"".join(struct.pack("<h", sample) for sample in samples)
        wav_file.writeframes(frame_data)

def play_wave(path: str) -> None:
    try:
        subprocess.run(["aplay", path], check=True)
    except FileNotFoundError:
        print("没有找到 aplay，请先执行: sudo apt install alsa-utils")
        sys.exit(1)
    except subprocess.CalledProcessError as exc:
        print(f"播放失败，退出码: {exc.returncode}")
        sys.exit(exc.returncode)

def main() -> None:
    parser = argparse.ArgumentParser(description="在 Raspberry Pi 5 上生成并播放狗叫声")
    parser.add_argument("--output", default="dog_bark.wav", help="生成的 wav 文件路径")
    parser.add_argument("--no-play", action="store_true", help="只生成，不播放")
    args = parser.parse_args()

    if not os.path.exists(args.output):
        samples = make_bark_waveform()
        write_wave(args.output, samples)
        print(f"已生成: {os.path.abspath(args.output)}")

    if not args.no_play:
        play_wave(args.output)

if __name__ == "__main__":
    main()
#'@ | ssh rdrp@192.168.137.220 "cat > ~/dog_bark_pi5.py"

#ssh rdrp@192.168.137.220 "sudo apt-get update && sudo apt-get install -y alsa-utils && python3 ~/dog_bark_pi5.py --no-play"

