import pytest
from PIL import Image

from macgyvbot_perception.grasp_point.vlm import models


torch = pytest.importorskip("torch")


class ProcessorCallFake:
    def __init__(self):
        self.chat_calls = []
        self.processor_calls = []

    def apply_chat_template(self, messages, **kwargs):
        self.chat_calls.append(kwargs)
        assert kwargs["tokenize"] is False
        return "prompt text"

    def __call__(self, **kwargs):
        self.processor_calls.append(kwargs)
        return {"input_ids": torch.tensor([[1, 2, 3]])}


class TokenizeFallbackFake:
    def __init__(self):
        self.tokenize_kwargs = None

    def apply_chat_template(self, messages, **kwargs):
        if kwargs["tokenize"] is False:
            return "prompt text"
        self.tokenize_kwargs = kwargs
        return torch.tensor([[4, 5]])

    def __call__(self, **kwargs):
        raise TypeError("processor call unsupported")


def test_build_inputs_uses_processor_call_and_adds_attention_mask():
    vlm = models.VLMOnly()
    vlm.processor = ProcessorCallFake()
    image = Image.new("RGB", (10, 10))

    inputs = vlm._build_inputs(image, "choose grasp")

    assert "attention_mask" in inputs
    assert inputs["attention_mask"].tolist() == [[1, 1, 1]]
    assert vlm.processor.processor_calls[0]["return_tensors"] == "pt"
    assert vlm.processor.processor_calls[0]["padding"] is True


def test_build_inputs_fallback_does_not_pass_return_dict():
    vlm = models.VLMOnly()
    vlm.processor = TokenizeFallbackFake()
    image = Image.new("RGB", (10, 10))

    inputs = vlm._build_inputs(image, "choose grasp")

    assert "return_dict" not in vlm.processor.tokenize_kwargs["processor_kwargs"]
    assert inputs["input_ids"].tolist() == [[4, 5]]
    assert inputs["attention_mask"].tolist() == [[1, 1]]
