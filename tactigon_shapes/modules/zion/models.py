#********************************************************************************
# Copyright (c) 2025 Next Industries s.r.l.
#
# This program and the accompanying materials are made available under the
# terms of the Apache 2.0 which is available at http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0
#
# Project Name:
# Tactigon Soul - Shape
# 
# Release date: 30/09/2025
# Release version: 1.0
#
# Contributors:
# - Massimiliano Bellino
# - Stefano Barbareschi
#********************************************************************************/


from enum import Enum
from dataclasses import dataclass


class Scope(Enum):
    SERVER = "SERVER_SCOPE"
    CLIENT = "CLIENT_SCOPE"
    SHARED = "SHARED_SCOPE"

class AlarmStatus(Enum):
    CLEARED_UNACK = "CLEARED_UNACK"

class AlarmSearchStatus(Enum):
    ACK = "ACK"
    ACTIVE = "ACTIVE"
    ANY = "ANY"
    CLEARED = "CLEARED"
    UNACK = "UNACK"

class AlarmSeverity(Enum):
    ANY = ""
    CRITICAL = "CRITICAL"

@dataclass
class Id:
    entityType: str
    id: str

@dataclass
class Device:
    id: Id
    name: str
    type: str
    tenantId: Id
    customerId: Id

    @classmethod
    def FromZION(cls, json):
        return cls(
            Id(json["id"]["entityType"],json["id"]["id"]),
            json["name"],
            json["type"],
            Id(json["tenantId"]["entityType"],json["tenantId"]["id"]),
            Id(json["customerId"]["entityType"],json["customerId"]["id"]),
        )

    def to_alarm(self) -> dict:
        return {
            "tenantId": {
                "entityType": self.tenantId.entityType,
                "id": self.tenantId.id
            },
            "customerId": {
                "entityType": self.customerId.entityType,
                "id": self.customerId.id
            },
            "originator": {
                "entityType": self.id.entityType,
                "id": self.id.id
            }
        }
    
@dataclass
class DeviceAlarm:
    id: str
    severity: AlarmSeverity
    status: AlarmStatus
    startTs: int

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            json["id"],
            AlarmSeverity[json["severity"]],
            AlarmStatus[json["status"]],
            json["startTs"]
        )
    
    def toJSON(self) -> object:
        return {
            "id": self.id,
            "severity": self.severity.value,
            "status": self.status.value,
            "startTs": self.startTs
        }

@dataclass
class ZionConfig:
    username: str
    password: str
    url: str = "https://zion.nextind.eu/"
    token: str = ""

    @classmethod
    def Default(cls):
        return cls(
            "",
            "",
            ZionConfig.url
        )

    @classmethod
    def FromJSON(cls, json: dict):
        return cls(
            json["username"],
            json["password"],
            json["url"] if "url" in json and json["url"] else cls.url
        )
    
    def toJSON(self) -> object:
        return {
            "username": self.username,
            "password": self.password,
            "url": self.url
        }
    
    def is_valid(self) -> bool:
        return self.username != "" and self.password != "" and self.url != ""