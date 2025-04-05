import detector
import frame_source

if __name__ == '__main__':
    imageSource = frame_source.ImageFileSource()

    while True:
        frame = imageSource.get_frame()
        if frame is None:
            break

        arena_selector = detector.ArenaDetector(frame)