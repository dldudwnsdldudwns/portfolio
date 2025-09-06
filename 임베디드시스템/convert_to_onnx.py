from transformers import AutoTokenizer, AutoModelForSequenceClassification
import torch

model = AutoModelForSequenceClassification.from_pretrained("michellejieli/emotion_text_classifier")
tokenizer = AutoTokenizer.from_pretrained("michellejieli/emotion_text_classifier")

inputs = tokenizer("This is a test", return_tensors="pt")


torch.onnx.export(
    model,
    (inputs["input_ids"],),
    "emotion.onnx",
    input_names=["input_ids"],
    output_names=["logits"],
    dynamic_axes={
        "input_ids": {0: "batch_size", 1: "sequence_length"},
        "logits": {0: "batch_size"}
    },
    opset_version=14
)