from typing import Dict, Type, List

from sketch_runtime.base_skill import BaseSkill
from sketch_runtime.task_context import TaskContext


class SkillRegistry:
    _skills: Dict[str, Type[BaseSkill]] = {}

    @classmethod
    def register(cls, skill_cls: Type[BaseSkill]):
        if not hasattr(skill_cls, "name") or not skill_cls.name:
            raise ValueError(f"Skill class {skill_cls} must define 'name'")
        cls._skills[skill_cls.name] = skill_cls

    @classmethod
    def get(cls, name: str) -> Type[BaseSkill]:
        return cls._skills.get(name)

    @classmethod
    def list_all(cls) -> List[str]:
        return list(cls._skills.keys())

    @classmethod
    def has(cls, name: str) -> bool:
        return name in cls._skills

    @classmethod
    def clear(cls):
        cls._skills.clear()


class SkillManager:
    INTENT_MAP = {
        "pick": "pick_skill",
        "place": "place_skill",
        "pour": "pour_skill",
        "move": "move_skill",
        "hold": "pick_skill",
        "grasp": "pick_skill",
        "release": "place_skill",
        "home": "home_skill",
    }

    def __init__(self, adapter=None):
        self.adapter = adapter

    def select(self, ctx: TaskContext) -> str:
        intent = ""
        if ctx.parsed_command:
            intent = (ctx.parsed_command.get("action") or "").lower()
        if not intent and ctx.target_object:
            intent = (ctx.target_object.get("intent") or "").lower()
        return self.INTENT_MAP.get(intent, "unknown_skill")

    def instantiate(self, skill_name: str) -> BaseSkill:
        skill_cls = SkillRegistry.get(skill_name)
        if skill_cls is None:
            return None
        return skill_cls(self.adapter)

    def select_and_instantiate(self, ctx: TaskContext) -> BaseSkill:
        name = self.select(ctx)
        return self.instantiate(name)
