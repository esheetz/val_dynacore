# adapted from https://realpython.com/python-speech-recognition/
import speech_recognition as sr

def initializeSpeechRecognition():
    r = sr.Recognizer()
    mic = sr.Microphone()
    return r, mic

def speechToText(r, mic):
    success = False;
    while not success:

        with mic as source:
            r.adjust_for_ambient_noise(source)
            print("What is your command?")
            audio = r.listen(source)

        try:
            transcription = r.recognize_google(audio)  # transcribe audio
        except sr.RequestError:
            # API was unreachable or unresponsive
            print("API unavailable")
            success = False
            continue
        except sr.UnknownValueError:
            # speech was unintelligible
            print("Unable to recognize speech, try again")
            success = False
            continue

        print("I heard:")
        print(transcription)
        text = input("Is this correct y/n: ")
        if text == "y":
            success = True
        else:
            success = False
    return transcription



