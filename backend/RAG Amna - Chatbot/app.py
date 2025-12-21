from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from qdrant_client import QdrantClient
import cohere
from dotenv import load_dotenv
import os

load_dotenv()

# --- 1. CONFIGURATION ---
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COHERE_KEY = os.getenv("COHERE_KEY")
COLLECTION_NAME = "physical_ai_book_cohere"

# --- 2. CLIENTS SETUP ---
# Qdrant
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Cohere
co = cohere.Client(COHERE_KEY)

# --- 3. FASTAPI APP ---
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- 4. LOGIC ---
@app.post("/chat")
async def chat_endpoint(request: Request):
    data = await request.json()
    user_query = data.get("query", "")
    
    if not user_query: return {"answer": "Please ask a question."}

    # Step A: Search Context
    context = ""
    try:
        # Embed query
        embed_resp = co.embed(
            texts=[user_query],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        # Search Qdrant
        hits = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=embed_resp.embeddings[0],
            limit=3
        )
        if hits:
            context = "\n\n".join([hit.payload['text'] for hit in hits])
    except Exception as e:
        print(f"Search Error: {e}")
        context = "No context found due to connection error."

    # Step B: Generate Answer (UPDATED MODEL NAME HERE)
    try:
        if context and len(context) > 10:
            system_prompt = f"You are a helpful AI Tutor for a book on Physical AI. Answer the question based ONLY on the context provided below.\n\nContext:\n{context}"
        else:
            system_prompt = "You are a helpful AI Tutor. Answer the user's question generally."

      
        response = co.chat(
            message=user_query,
            model="command-a-03-2025", 
            preamble=system_prompt,
            temperature=0.3
        )
        
        answer = response.text

    except Exception as e:
      
        try:
             response = co.chat(
                message=user_query,
                model="command-light", 
                preamble=system_prompt,
                temperature=0.3
            )
             answer = response.text
        except:
             answer = f"Error generating answer: {str(e)}"

    return {"answer": answer}

@app.get("/")
def home():
    return {"status": "Running", "model": "command-a-03-2025"}