const input = document.getElementById("userInput");
const sendBtn = document.getElementById("sendBtn");
const chat = document.getElementById("chat");
const modelSelect = document.getElementById("modelSelect");

/* Auto-grow textarea */
input.addEventListener("input", () => {
  input.style.height = "auto";
  input.style.height = input.scrollHeight + "px";
});

/* Enter = Send | Shift+Enter = New line */
input.addEventListener("keydown", (e) => {
  if (e.key === "Enter" && !e.shiftKey) {
    e.preventDefault();
    sendMessage();
  }
});

sendBtn.addEventListener("click", sendMessage);

function addMessage(text, sender) {
  const msg = document.createElement("div");
  msg.classList.add("message", sender);
  msg.innerText = text;
  chat.appendChild(msg);
  chat.scrollTop = chat.scrollHeight;
  return msg;
}

/* Loader */
function addLoader() {
  const msg = document.createElement("div");
  msg.classList.add("message", "bot", "loader-bubble");
  msg.innerHTML = `
    <span class="loader-dot">.</span>
    <span class="loader-dot">.</span>
    <span class="loader-dot">.</span>
  `;
  chat.appendChild(msg);
  chat.scrollTop = chat.scrollHeight;
  return msg;
}

/* Typing animation */
function typeText(element, text, speed = 35) {
  element.innerText = "";
  const tokens = text.match(/\S+|\s+/g) || [];
  let i = 0;

  const interval = setInterval(() => {
    if (i < tokens.length) {
      element.innerText += tokens[i];
      chat.scrollTop = chat.scrollHeight;
      i++;
    } else {
      clearInterval(interval);
    }
  }, speed);
}

async function sendMessage() {
  const text = input.value.trim();
  if (!text) return;

  addMessage(text, "user");
  input.value = "";
  input.style.height = "auto";

  const loader = addLoader();

  const response = await fetch("/generate", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      prompt: text,
      model: modelSelect.value
    })
  });

  const data = await response.json();
  chat.removeChild(loader);

  const botMsg = document.createElement("div");
  botMsg.classList.add("message", "bot");
  chat.appendChild(botMsg);

  typeText(botMsg, data.response);
}
