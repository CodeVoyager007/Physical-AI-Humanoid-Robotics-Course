# Data Model: Hackathon Bonus Pack

## User

Represents a user of the platform.

| Field | Type | Description |
|---|---|---|
| id | UUID | Primary key |
| email | String | User's email address (must be unique) |
| password | String | Hashed password |
| software_background | String | User's software background (Beginner, Python Dev, C++ Dev) |
| hardware_background | String | User's hardware background (None, Arduino, Jetson) |

## Chapter

Represents a single chapter of the book. This is not a database entity but a conceptual one for the API.

| Field | Type | Description |
|---|---|---|
| text | String | The full text content of the chapter. |
