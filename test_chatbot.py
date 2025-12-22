import requests
import json

# Test the chatbot query endpoint
url = "http://127.0.0.1:8000/v1/query"
payload = {"query": "What is physical AI?"}

print("Testing chatbot API endpoint...")
print(f"URL: {url}")
print(f"Payload: {json.dumps(payload, indent=2)}")
print("\nSending request...")

try:
    response = requests.post(url, json=payload, timeout=30)
    print(f"\nStatus Code: {response.status_code}")
    print(f"Headers: {dict(response.headers)}")
    print(f"\nResponse Content Length: {len(response.content)} bytes")
    print(f"Response Text (first 1000 chars):\n{response.text[:1000]}")

    if response.status_code == 200:
        try:
            data = response.json()
            print("\n=== PARSED JSON RESPONSE ===")
            print(json.dumps(data, indent=2))
        except Exception as e:
            print(f"\nFailed to parse JSON: {e}")
            print(f"Raw content: {response.content}")

except Exception as e:
    print(f"\nError: {e}")
