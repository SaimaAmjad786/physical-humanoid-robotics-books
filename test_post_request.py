#!/usr/bin/env python3
import requests
import json

url = "http://127.0.0.1:8000/v1/query"
headers = {
    "Content-Type": "application/json",
    "Origin": "http://localhost:3000"
}
payload = {"query": "What is physical AI?"}

print(f"Testing POST {url}")
print(f"Headers: {json.dumps(headers, indent=2)}")
print(f"Payload: {json.dumps(payload, indent=2)}")
print("\n" + "="*50)

try:
    response = requests.post(url, json=payload, headers=headers, timeout=10, stream=False)

    print(f"\nStatus Code: {response.status_code}")
    print(f"Response Headers:")
    for k, v in response.headers.items():
        print(f"  {k}: {v}")

    print(f"\nContent Length: {len(response.content)}")
    print(f"Text Length: {len(response.text)}")

    if response.content:
        print(f"\nRaw Content (first 500 bytes):")
        print(response.content[:500])

        print(f"\nText Content:")
        print(response.text[:500])

        try:
            data = response.json()
            print(f"\n=== PARSED JSON ===")
            print(json.dumps(data, indent=2))
        except Exception as e:
            print(f"\nFailed to parse JSON: {e}")
    else:
        print("\n⚠️  RESPONSE IS EMPTY!")

except Exception as e:
    print(f"\n❌ ERROR: {e}")
    import traceback
    traceback.print_exc()
