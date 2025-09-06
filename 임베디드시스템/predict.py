import subprocess
import onnxruntime as ort
from transformers import AutoTokenizer
import numpy as np

tokenizer = AutoTokenizer.from_pretrained("michellejieli/emotion_text_classifier")

session = ort.InferenceSession("emotion.onnx")
labels = ['anger', 'disgust', 'fear', 'joy', 'neutral', 'sadness' , 'surprise']
label_to_number = {
    'neutral': 0,
    'joy': 1,
    'sadness': 2,
    'anger': 3
}
def predict(text):
    inputs = tokenizer(text, return_tensors="np", padding = True,truncation=True)
    input_ids = inputs["input_ids"].astype(np.int64)
    ort_inputs = {"input_ids": input_ids}
    logits = session.run(None,ort_inputs)[0]

    label_id = int(np.argmax(logits))
    return 'anger' if labels[label_id] in ['disgust', 'fear', 'surprise'] else labels[label_id]

def run_program_by_label(label):
    if label not in label_to_number:
        print(f"'{label}' is error.")
        return

    number = str(label_to_number[label])
    command = ["sudo","./led7seg", number]
    try:
        subprocess.run(command, check=True)
        print(f"execution : ./led7seg {number}")
    except subprocess.CalledProcessError as e:
        print(f"error : {e}")

if __name__ == "__main__":
    while True:
        input_text = input(": ")
        emotion = predict(input_text)
        print(f"RESULT : {emotion}")
        run_program_by_label(emotion)
