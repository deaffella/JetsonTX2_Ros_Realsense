from transliterate import translit



def translit_ru_to_eng(ru_text: str) -> str:
    return translit(ru_text, language_code='ru', reversed=True)

