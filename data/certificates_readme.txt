AWS IoT Core Certificates Setup
=====================================

You need to place three certificate files in this data/ directory:

1. ca-cert.pem - Amazon Root CA 1 Certificate
2. device-cert.pem - Your AWS IoT device certificate  
3. private-key.pem - Your AWS IoT device private key

How to obtain these certificates:
---------------------------------

1. ROOT CA CERTIFICATE (ca-cert.pem):
   Download from: https://www.amazontrust.com/repository/AmazonRootCA1.pem
   
   Or use curl:
   curl -o ca-cert.pem https://www.amazontrust.com/repository/AmazonRootCA1.pem

2. DEVICE CERTIFICATE and PRIVATE KEY:
   a) Go to AWS IoT Core console
   b) Navigate to Manage > Things
   c) Create a new Thing (or select existing)
   d) Go to Security > Certificates
   e) Create a new certificate
   f) Download the certificate file and rename to: device-cert.pem
   g) Download the private key file and rename to: private-key.pem
   h) Attach the certificate to your Thing
   i) Create and attach an IoT policy to the certificate

Example IoT Policy:
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": [
        "iot:Connect",
        "iot:Publish",
        "iot:Subscribe",
        "iot:Receive"
      ],
      "Resource": "*"
    }
  ]
}

Security Notes:
- Never commit real certificate files to version control
- Keep your private key secure
- Use appropriate IoT policies to restrict access
- Consider using AWS IoT Device Management for production deployments

File Structure After Setup:
data/
├── ca-cert.pem
├── device-cert.pem
└── private-key.pem 